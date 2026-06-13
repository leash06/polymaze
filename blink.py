from machine import Pin, PWM
import time
import array

# PIN DEFINITIONS
QTR_PINS = [Pin(i, Pin.IN) for i in range(2, 10)]
LEDON = Pin(10, Pin.OUT)
AIN1, AIN2 = Pin(11, Pin.OUT), Pin(12, Pin.OUT)
PWMA = PWM(Pin(13))
BIN1, BIN2 = Pin(14, Pin.OUT), Pin(15, Pin.OUT)
PWMB = PWM(Pin(16))
LED_R, LED_G = Pin(17, Pin.OUT), Pin(18, Pin.OUT)

# CONSTANTS
NUM_SENSORS = 8
TIMEOUT_US = 2500
BLACK_THRESHOLD = 1500
SPEED_NORMAL = 40000
SPEED_TURN = 35000
MOVE_TIME = 0.4
TURN_TIME = 0.35
KP, KI, KD = 0.8, 0.02, 0.3

PWMA.freq(1000)
PWMB.freq(1000)

# CALIBRATION & STATE
calibration_min = array.array('I', [TIMEOUT_US] * NUM_SENSORS)
calibration_max = array.array('I', [0] * NUM_SENSORS)
pid_integral = pid_last_error = 0
path = []

def read_sensor(pin):
    pin.init(Pin.OUT)
    pin.value(1)
    time.sleep_us(10)
    pin.init(Pin.IN)
    start = time.ticks_us()
    while pin.value() == 1:
        if time.ticks_diff(time.ticks_us(), start) > TIMEOUT_US:
            return TIMEOUT_US
    return time.ticks_diff(time.ticks_us(), start)

def read_all_sensors():
    return [read_sensor(pin) for pin in QTR_PINS]

def calibrate_sensors(duration=2.0):
    print("Calibrating sensors...")
    LEDON.value(1)
    start_time = time.ticks_ms()
    samples = 0
    
    while time.ticks_diff(time.ticks_ms(), start_time) < duration * 1000:
        values = read_all_sensors()
        for i in range(NUM_SENSORS):
            calibration_min[i] = min(calibration_min[i], values[i])
            calibration_max[i] = max(calibration_max[i], values[i])
        samples += 1
        time.sleep_ms(10)
    
    print(f"Calibration complete: {samples} samples")
    print(f"Min: {list(calibration_min)}")
    print(f"Max: {list(calibration_max)}")

def get_calibrated_position():
    raw_values = read_all_sensors()
    calibrated = [0] * NUM_SENSORS
    
    for i in range(NUM_SENSORS):
        if calibration_max[i] > calibration_min[i]:
            val = (raw_values[i] - calibration_min[i]) * 1000
            calibrated[i] = val // (calibration_max[i] - calibration_min[i])
            calibrated[i] = max(0, min(1000, 1000 - calibrated[i]))
    
    weighted_sum = sum(i * calibrated[i] for i in range(NUM_SENSORS))
    total = sum(calibrated)
    
    return 0 if total == 0 else (weighted_sum * 2000) // (total * (NUM_SENSORS - 1)) - 1000

def is_on_line():
    return any(v > BLACK_THRESHOLD for v in read_all_sensors())

def is_intersection():
    black_count = sum(1 for v in read_all_sensors() if v > BLACK_THRESHOLD)
    return black_count >= 6

def is_finish():
    black_count = 0
    start = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), start) < 500:
        if all(v > BLACK_THRESHOLD for v in read_all_sensors()):
            black_count += 1
        time.sleep_ms(20)
    return black_count >= 10

def set_motors(left_speed, right_speed):
    AIN1.value(1 if left_speed >= 0 else 0)
    AIN2.value(0 if left_speed >= 0 else 1)
    BIN1.value(1 if right_speed >= 0 else 0)
    BIN2.value(0 if right_speed >= 0 else 1)
    PWMA.duty_u16(min(abs(left_speed), 65535))
    PWMB.duty_u16(min(abs(right_speed), 65535))

def stop_motors():
    set_motors(0, 0)

def move_forward():
    set_motors(SPEED_NORMAL, SPEED_NORMAL)
    time.sleep(MOVE_TIME)
    stop_motors()

def move_backward():
    set_motors(-SPEED_NORMAL, -SPEED_NORMAL)
    time.sleep(MOVE_TIME)
    stop_motors()

def turn_left():
    set_motors(-SPEED_TURN, SPEED_TURN)
    time.sleep(TURN_TIME)
    stop_motors()

def turn_right():
    set_motors(SPEED_TURN, -SPEED_TURN)
    time.sleep(TURN_TIME)
    stop_motors()

def turn_around():
    turn_left()
    turn_left()

def turn_left_small():
    set_motors(-SPEED_TURN//2, SPEED_TURN//2)
    time.sleep(TURN_TIME / 2)
    stop_motors()

def turn_right_small():
    set_motors(SPEED_TURN//2, -SPEED_TURN//2)
    time.sleep(TURN_TIME / 2)
    stop_motors()

def follow_line():
    global pid_integral, pid_last_error
    position = get_calibrated_position()
    error = position / 1000.0
    
    pid_integral += error * 0.01
    pid_integral = max(-1.0, min(1.0, pid_integral))
    
    derivative = (error - pid_last_error) / 0.01
    correction = KP * error + KI * pid_integral + KD * derivative
    pid_last_error = error
    
    base_speed = SPEED_NORMAL
    left_speed = max(0, min(65535, base_speed - int(correction * 20000)))
    right_speed = max(0, min(65535, base_speed + int(correction * 20000)))
    set_motors(left_speed, right_speed)

def move_to_intersection():
    while not is_intersection():
        follow_line()
        time.sleep(0.005)
    stop_motors()

def move_to_intersection_sprint():
    last_check = 0
    while True:
        follow_line()
        if time.ticks_diff(time.ticks_ms(), last_check) > 20:
            if is_intersection():
                break
            last_check = time.ticks_ms()
        time.sleep(0.005)
    stop_motors()

def left_path_exists():
    turn_left_small()
    time.sleep(0.1)
    exists = is_on_line()
    turn_right_small()
    return exists

def straight_path_exists():
    values = read_all_sensors()
    return values[3] > BLACK_THRESHOLD or values[4] > BLACK_THRESHOLD

def right_path_exists():
    turn_right_small()
    time.sleep(0.1)
    exists = is_on_line()
    turn_left_small()
    return exists

def get_available_paths():
    available = []
    if left_path_exists():
        available.append('L')
    if straight_path_exists():
        available.append('S')
    if right_path_exists():
        available.append('R')
    return available

def optimize_path(raw_path):
    optimized = []
    for move in raw_path:
        if move == 'B' and optimized:
            optimized.pop()
        else:
            optimized.append(move)
    return optimized

def explore_maze():
    global path
    path = []
    print("\n=== PHASE 1: EXPLORING MAZE ===")
    step = 0
    
    while True:
        step += 1
        print(f"Step {step}")
        move_to_intersection()
        
        if is_finish():
            print("FINISH REACHED!")
            return True
        
        available = get_available_paths()
        print(f"Available: {available}")
        
        if 'L' in available:
            choice = 'L'
            turn_left()
        elif 'S' in available:
            choice = 'S'
        elif 'R' in available:
            choice = 'R'
            turn_right()
        else:
            choice = 'B'
            turn_around()
        
        path.append(choice)
        move_forward()

def sprint_maze(optimized_path):
    print("\n=== PHASE 2: SPRINTING ===")
    for i, move in enumerate(optimized_path):
        print(f"Move {i+1}: {move}")
        if move == 'L':
            turn_left()
        elif move == 'R':
            turn_right()
        move_to_intersection_sprint()
    print("MAZE SOLVED!")

def led_exploring():
    LED_R.value(1)
    LED_G.value(0)

def led_sprinting():
    LED_R.value(0)
    LED_G.value(1)

def led_victory():
    for _ in range(5):
        LED_G.value(1)
        time.sleep(0.2)
        LED_G.value(0)
        time.sleep(0.2)

def led_error():
    for _ in range(3):
        LED_R.value(1)
        time.sleep(0.1)
        LED_R.value(0)
        time.sleep(0.1)

def main():
    print("\n" + "="*50)
    print("POLYMAZE 2026 - Maze Solving Robot")
    print("="*50)
    
    LEDON.value(1)
    print("\nWave robot over black/white lines")
    input("Press Enter to calibrate...")
    calibrate_sensors(duration=3.0)
    
    print("\nTesting calibration...")
    for _ in range(5):
        print(f"Position: {get_calibrated_position()}")
        time.sleep(0.2)
    
    print("Calibration OK!\n")
    
    led_exploring()
    explore_maze()
    
    print(f"Raw path: {path}")
    optimized = optimize_path(path)
    print(f"Optimized: {optimized}")
    
    if not optimized:
        print("No path found!")
        led_error()
        return
    
    led_sprinting()
    sprint_maze(optimized)
    led_victory()
    
    print("\n" + "="*50)
    print("MISSION ACCOMPLISHED!")
    print("="*50)
    
    while True:
        LED_G.value(1)
        time.sleep(0.5)
        LED_G.value(0)
        time.sleep(0.5)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nStopped by user")
        stop_motors()
        LEDON.value(0)
        LED_R.value(0)
        LED_G.value(0)
    except Exception as e:
        print(f"Error: {e}")
        stop_motors()
        LEDON.value(0)
