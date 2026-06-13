"""
POLYMAZE 2026 - Maze Solving Robot
Corrected Pins Based on Pico Pinout Diagram
Hardware: Raspberry Pi Pico + QTR-8RC + 2 Motors
"""

from machine import Pin, PWM
import time
import array

# ==================== CORRECT PIN DEFINITIONS ====================
# QTR-8RC Sensor Pins (based on diagram)
# Using a mix of ADC and digital pins as available
QTR_PINS = [
    Pin(26, Pin.IN),   # OUT1 - far left (ADC capable)
    Pin(27, Pin.IN),   # OUT2 (ADC capable)
    Pin(28, Pin.IN),   # OUT3 (ADC capable)
    Pin(2, Pin.IN),    # OUT4
    Pin(3, Pin.IN),    # OUT5
    Pin(4, Pin.IN),    # OUT6
    Pin(5, Pin.IN),    # OUT7
    Pin(6, Pin.IN),    # OUT8 - far right
]

# LEDON pin (controls IR LEDs on QTR-8RC)
LEDON = Pin(7, Pin.OUT)  # GP7

# Motor Driver Pins (TB6612FNG)
AIN1 = Pin(8, Pin.OUT)   # Left motor direction 1
AIN2 = Pin(9, Pin.OUT)   # Left motor direction 2
PWMA = PWM(Pin(10))      # Left motor speed (PWM)

BIN1 = Pin(11, Pin.OUT)  # Right motor direction 1
BIN2 = Pin(12, Pin.OUT)  # Right motor direction 2
PWMB = PWM(Pin(13))      # Right motor speed (PWM)

# Status LEDs (optional)
LED_R = Pin(14, Pin.OUT)  # Red LED
LED_G = Pin(15, Pin.OUT)  # Green LED

# ==================== CONSTANTS ====================
NUM_SENSORS = 8
TIMEOUT_US = 2500           # Timeout for dark surfaces (microseconds)
BLACK_THRESHOLD = 1500      # Values above this = black line

# Motor speeds (PWM 0-65535)
SPEED_NORMAL = 40000        # Normal forward speed
SPEED_TURN = 35000          # Turning speed

# Timing constants (CALIBRATE THESE for your robot!)
MOVE_TIME = 0.4             # Time to move one cell (seconds)
TURN_TIME = 0.35            # Time for 90° turn (seconds)

# PID Constants for smooth line following
KP = 0.8                    # Proportional gain
KI = 0.02                   # Integral gain
KD = 0.3                    # Derivative gain

# Setup PWM frequency
PWMA.freq(1000)
PWMB.freq(1000)

# ==================== CALIBRATION STORAGE ====================
calibration_min = array.array('I', [TIMEOUT_US] * NUM_SENSORS)
calibration_max = array.array('I', [0] * NUM_SENSORS)

# ==================== PID VARIABLES ====================
pid_integral = 0
pid_last_error = 0

# ==================== PATH STORAGE ====================
path = []


# ==================== QTR-8RC FUNCTIONS ====================

def read_sensor(pin):
    """
    Read one QTR-8RC sensor using RC timing.
    Returns microseconds (0 to TIMEOUT_US)
    Lower = lighter (white), Higher = darker (black)
    """
    # Charge the capacitor
    pin.init(Pin.OUT)
    pin.value(1)
    time.sleep_us(10)
    
    # Discharge and measure
    pin.init(Pin.IN)
    start = time.ticks_us()
    
    while pin.value() == 1:
        if time.ticks_diff(time.ticks_us(), start) > TIMEOUT_US:
            return TIMEOUT_US
    
    return time.ticks_diff(time.ticks_us(), start)

def read_all_sensors():
    """Read all 8 sensors, return list of values"""
    return [read_sensor(pin) for pin in QTR_PINS]

def calibrate_sensors(duration=2.0):
    """
    Calibrate sensors by waving robot over black and white.
    Call this at startup before maze solving.
    """
    print("Calibrating sensors... Wave robot over black and white lines")
    LEDON.value(1)  # Turn IR LEDs on
    
    start_time = time.ticks_ms()
    samples = 0
    
    while time.ticks_diff(time.ticks_ms(), start_time) < duration * 1000:
        values = read_all_sensors()
        for i in range(NUM_SENSORS):
            if values[i] < calibration_min[i]:
                calibration_min[i] = values[i]
            if values[i] > calibration_max[i]:
                calibration_max[i] = values[i]
        samples += 1
        time.sleep_ms(10)
    
    print(f"Calibration complete! {samples} samples")
    print(f"Min: {[calibration_min[i] for i in range(NUM_SENSORS)]}")
    print(f"Max: {[calibration_max[i] for i in range(NUM_SENSORS)]}")
    time.sleep(1)

def get_calibrated_position():
    """
    Get line position using calibrated values.
    Returns position from -1000 (far left) to +1000 (far right)
    0 = centered
    """
    raw_values = read_all_sensors()
    calibrated = [0] * NUM_SENSORS
    
    for i in range(NUM_SENSORS):
        if calibration_max[i] > calibration_min[i]:
            # Map to 0-1000 range (0=white, 1000=black)
            val = (raw_values[i] - calibration_min[i]) * 1000
            calibrated[i] = val // (calibration_max[i] - calibration_min[i])
            # Invert: black = high (for weighted average)
            calibrated[i] = 1000 - calibrated[i]
            # Clamp to 0-1000 range
            if calibrated[i] < 0:
                calibrated[i] = 0
            if calibrated[i] > 1000:
                calibrated[i] = 1000
    
    # Calculate weighted average
    weighted_sum = 0
    total = 0
    for i in range(NUM_SENSORS):
        weighted_sum += i * calibrated[i]
        total += calibrated[i]
    
    if total == 0:
        return 0
    
    # Convert to -1000 to +1000 range
    position = (weighted_sum * 2000) // (total * (NUM_SENSORS - 1)) - 1000
    return position

def is_on_line():
    """Check if robot is currently on a black line"""
    values = read_all_sensors()
    return any(v > BLACK_THRESHOLD for v in values)

def is_intersection():
    """Check if at intersection (most sensors see black)"""
    values = read_all_sensors()
    black_count = sum(1 for v in values if v > BLACK_THRESHOLD)
    return black_count >= 6

def is_finish():
    """Check if at finish (all sensors black for 0.5 seconds)"""
    black_count = 0
    start = time.ticks_ms()
    
    while time.ticks_diff(time.ticks_ms(), start) < 500:
        values = read_all_sensors()
        if all(v > BLACK_THRESHOLD for v in values):
            black_count += 1
        time.sleep_ms(20)
    
    return black_count >= 10


# ==================== MOTOR CONTROL ====================

def set_motors(left_speed, right_speed):
    """
    Set motor speeds (-65535 to 65535)
    Negative = reverse, Positive = forward
    """
    # Left motor direction
    if left_speed >= 0:
        AIN1.value(1)
        AIN2.value(0)
    else:
        AIN1.value(0)
        AIN2.value(1)
        left_speed = -left_speed
    
    # Right motor direction
    if right_speed >= 0:
        BIN1.value(1)
        BIN2.value(0)
    else:
        BIN1.value(0)
        BIN2.value(1)
        right_speed = -right_speed
    
    # Apply speeds (clamp to 0-65535)
    PWMA.duty_u16(min(abs(left_speed), 65535))
    PWMB.duty_u16(min(abs(right_speed), 65535))

def stop_motors():
    """Stop both motors"""
    set_motors(0, 0)

def move_forward():
    """Move forward one cell"""
    set_motors(SPEED_NORMAL, SPEED_NORMAL)
    time.sleep(MOVE_TIME)
    stop_motors()

def move_backward():
    """Move backward one cell"""
    set_motors(-SPEED_NORMAL, -SPEED_NORMAL)
    time.sleep(MOVE_TIME)
    stop_motors()

def turn_left():
    """Turn 90 degrees left"""
    set_motors(-SPEED_TURN, SPEED_TURN)
    time.sleep(TURN_TIME)
    stop_motors()

def turn_right():
    """Turn 90 degrees right"""
    set_motors(SPEED_TURN, -SPEED_TURN)
    time.sleep(TURN_TIME)
    stop_motors()

def turn_around():
    """Turn 180 degrees"""
    turn_left()
    turn_left()

def turn_left_small():
    """Small left turn for path checking"""
    set_motors(-SPEED_TURN//2, SPEED_TURN//2)
    time.sleep(TURN_TIME / 2)
    stop_motors()

def turn_right_small():
    """Small right turn for path checking"""
    set_motors(SPEED_TURN//2, -SPEED_TURN//2)
    time.sleep(TURN_TIME / 2)
    stop_motors()


# ==================== PID LINE FOLLOWING ====================

def follow_line():
    """
    PID line following - keeps robot centered on black line
    """
    global pid_integral, pid_last_error
    
    # Get current position (-1000 to +1000)
    position = get_calibrated_position()
    error = position / 1000.0  # Convert to -1..1 range
    
    # Proportional term (how far off now)
    proportional = error
    
    # Integral term (how long have we been off)
    pid_integral += error * 0.01
    # Anti-windup: limit integral
    if pid_integral > 1.0:
        pid_integral = 1.0
    elif pid_integral < -1.0:
        pid_integral = -1.0
    
    # Derivative term (how fast we're changing)
    derivative = (error - pid_last_error) / 0.01
    
    # Calculate total correction
    correction = KP * proportional + KI * pid_integral + KD * derivative
    
    # Store for next iteration
    pid_last_error = error
    
    # Apply correction to motor speeds
    base_speed = SPEED_NORMAL
    left_speed = base_speed - int(correction * 20000)
    right_speed = base_speed + int(correction * 20000)
    
    # Clamp to valid range
    left_speed = max(0, min(65535, left_speed))
    right_speed = max(0, min(65535, right_speed))
    
    set_motors(left_speed, right_speed)

def move_to_intersection():
    """Drive following line until reaching an intersection"""
    while not is_intersection():
        follow_line()
        time.sleep(0.005)  # 5ms update rate
    stop_motors()
    time.sleep(0.1)

def move_to_intersection_sprint():
    """Drive to intersection (faster, for sprint mode)"""
    last_check = 0
    while True:
        follow_line()
        # Check intersection every 20ms
        if time.ticks_diff(time.ticks_ms(), last_check) > 20:
            if is_intersection():
                break
            last_check = time.ticks_ms()
        time.sleep(0.005)
    stop_motors()
    time.sleep(0.05)


# ==================== PATH DETECTION ====================

def left_path_exists():
    """Check if line continues to the left"""
    turn_left_small()
    time.sleep(0.1)
    exists = is_on_line()
    turn_right_small()
    return exists

def straight_path_exists():
    """Check if line continues straight"""
    values = read_all_sensors()
    # Check center sensors (index 3 and 4)
    return values[3] > BLACK_THRESHOLD or values[4] > BLACK_THRESHOLD

def right_path_exists():
    """Check if line continues to the right"""
    turn_right_small()
    time.sleep(0.1)
    exists = is_on_line()
    turn_left_small()
    return exists

def get_available_paths():
    """Return list of available directions at current intersection"""
    available = []
    
    if left_path_exists():
        available.append('L')
    if straight_path_exists():
        available.append('S')
    if right_path_exists():
        available.append('R')
    
    return available

# ==================== PATH OPTIMIZATION ====================

def optimize_path(raw_path):
    """
    Remove dead ends from path.
    Example: ['L','S','B','R','R'] → ['L','R','R']
    When you see 'B', remove it and the previous move
    """
    optimized = []
    
    for move in raw_path:
        if move == 'B' and optimized:
            optimized.pop()  # Remove the move that led to dead end
        else:
            optimized.append(move)
    
    return optimized


# ==================== EXPLORATION MODE ====================

def explore_maze():
    """
    Phase 1: Explore entire maze using Left-Hand Rule
    Records all decisions in 'path' array
    """
    global path
    path = []
    
    print("\n=== PHASE 1: EXPLORING MAZE ===")
    print("Using Left-Hand Rule (L > S > R)")
    
    step = 0
    
    while True:
        step += 1
        print(f"\n--- Step {step} ---")
        
        # Move to next intersection
        move_to_intersection()
        
        # Check if reached finish
        if is_finish():
            print("\n🏁 FINISH REACHED! 🏁")
            return True
        
        # Check available paths
        available = get_available_paths()
        print(f"Available paths: {available}")
        
        # Choose using Left-Hand Rule: L > S > R
        if 'L' in available:
            choice = 'L'
            turn_left()
            print("Chose: LEFT")
            
        elif 'S' in available:
            choice = 'S'
            print("Chose: STRAIGHT")
            
        elif 'R' in available:
            choice = 'R'
            turn_right()
            print("Chose: RIGHT")
            
        else:
            choice = 'B'
            turn_around()
            print("Chose: DEAD END (turning around)")
        
        # Record decision
        path.append(choice)
        print(f"Path so far: {path}")
        
        # Move to next cell
        move_forward()

# ==================== SPRINT MODE ====================

def sprint_maze(optimized_path):
    """
    Phase 2: Execute optimized path to finish
    """
    print("\n=== PHASE 2: SPRINTING TO FINISH ===")
    print(f"Optimized path: {optimized_path}")
    
    for i, move in enumerate(optimized_path):
        print(f"\nMove {i+1}: {move}")
        
        if move == 'L':
            turn_left()
        elif move == 'R':
            turn_right()
        # 'S' = straight, do nothing
        
        # Move to next intersection
        move_to_intersection_sprint()
    
    print("\n🏆 MAZE SOLVED! VICTORY! 🏆")


# ==================== LED INDICATORS ====================

def led_exploring():
    """Red LED on = exploring mode"""
    LED_R.value(1)
    LED_G.value(0)

def led_sprinting():
    """Green LED on = sprinting mode"""
    LED_R.value(0)
    LED_G.value(1)

def led_victory():
    """Flash green LED = victory"""
    for _ in range(5):
        LED_G.value(1)
        time.sleep(0.2)
        LED_G.value(0)
        time.sleep(0.2)

def led_error():
    """Flash red LED = error"""
    for _ in range(3):
        LED_R.value(1)
        time.sleep(0.1)
        LED_R.value(0)
        time.sleep(0.1)


# ==================== MAIN PROGRAM ====================

def main():
    print("\n" + "="*50)
    print("   POLYMAZE 2026 - Maze Solving Robot")
    print("   QTR-8RC Sensor + Raspberry Pi Pico")
    print("="*50)
    
    # Turn on QTR-8RC IR LEDs
    LEDON.value(1)
    
    # Calibrate sensors
    print("\n⚠️  IMPORTANT: Wave the robot over black and white lines!")
    input("Press Enter when ready to calibrate...")
    calibrate_sensors(duration=3.0)
    
    # Test calibration
    print("\nTesting calibration...")
    for _ in range(5):
        pos = get_calibrated_position()
        print(f"Position: {pos}")
        time.sleep(0.2)
    
    print("\n✅ Calibration successful!")
    time.sleep(1)
    
    # Phase 1: Explore and memorize
    led_exploring()
    explore_maze()
    
    # Phase 2: Optimize path
    print(f"\n=== OPTIMIZING PATH ===")
    print(f"Raw path: {path}")
    optimized = optimize_path(path)
    print(f"Optimized path: {optimized}")
    
    if not optimized:
        print("❌ Error: No path found!")
        led_error()
        return
    
    # Phase 3: Sprint
    led_sprinting()
    sprint_maze(optimized)
    
    # Victory!
    led_victory()
    
    print("\n" + "="*50)
    print("   MISSION ACCOMPLISHED!")
    print("="*50)
    
    # Stay at finish with victory blink
    while True:
        LED_G.value(1)
        time.sleep(0.5)
        LED_G.value(0)
        time.sleep(0.5)

# ==================== RUN ====================
if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nProgram stopped by user")
        stop_motors()
        LEDON.value(0)
        LED_R.value(0)
        LED_G.value(0)
    except Exception as e:
        print(f"\n❌ Error: {e}")
        stop_motors()
        LEDON.value(0)
        LED_R.value(0)
        LED_G.value(0)
