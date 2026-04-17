# Import necessary modules
import time
from machine import Pin, SoftI2C
from motor import Motor
from ultrasonic import sonic
import ssd1306

# Setup sensors and motors
middle_IR = Pin(21, Pin.IN)
center_right_IR = Pin(22, Pin.IN)
center_left_IR = Pin(20, Pin.IN)
outer_right_IR = Pin(18, Pin.IN)
outer_left_IR = Pin(19, Pin.IN)
motor_left  = Motor("left", 11, 10, 7)
motor_right = Motor("right", 9, 8, 6)
ultrasonic_front = sonic(14, 15) 
ultrasonic_left  = sonic(3, 2)  
ultrasonic_right = sonic(12, 13)  

# Global counters
direction_counter = 0 # How many roundabouts it has been through

# Setup important constants
slow = 41
outsidewheel = 45
insidewheel = 35
collisiondist = 50 # mm 
centretollerance = 0.4 # tolerance for left/right ratio for no_line
pivottimeout = 3.0 # sec
ontime = 0.02
offtime = 0.01

# ------------------------------------------------------------------
# Enhanced Debounced Sensor Read with adjustable parameters
# ------------------------------------------------------------------
def read_ir_sensor(pin, samples=5, delay_ms=2):
    """
    Read IR sensor with majority voting to reject noise.
    Increasing samples improves stability but adds small latency.
    """
    count = 0
    for _ in range(samples):
        count += pin.value()
        time.sleep_ms(delay_ms)
    return 1 if count > samples // 2 else 0

# ------------------------------------------------------------------
# Initialise OLED
# ------------------------------------------------------------------
i2c = SoftI2C(scl=Pin(5), sda=Pin(4))
oled_width = 128
oled_height = 64
oled = ssd1306.SSD1306_I2C(oled_width, oled_height, i2c)
oled.invert(True)

def print_oled():
    oled.fill(0)
    oled.text("OL:" + str(outer_left_IR.value()), 0, 10)
    oled.text("CL:" + str(center_left_IR.value()), 45, 10)
    oled.text("M:" + str(middle_IR.value()), 85, 10)
    oled.text("CR:" + str(center_right_IR.value()), 0, 20)
    oled.text("OR:" + str(outer_right_IR.value()), 45, 20)
    oled.show()

# ------------------------------------------------------------------
# Collision Check
# ------------------------------------------------------------------
def check_collision():
    front_mm = ultrasonic_front.distance_mm()
    if front_mm < 0:
        print("ERROR: negative ultrasonic dist, continuing")
        return False
    elif front_mm < collisiondist:
        print(f"Collision: {front_mm} mm room left")
        stop()
        print_oled()
        oled.text(f"COLLISION: {front_mm}", 0, 40)
        oled.show()
        return True
    return False

# ------------------------------------------------------------------
# Basic Motor Actions
# ------------------------------------------------------------------
def drive_forward():
    motor_left.set_forwards()
    motor_right.set_forwards()
    motor_left.duty(slow)
    motor_right.duty(slow)

def stop():
    motor_left.duty(0)
    motor_right.duty(0)

# ------------------------------------------------------------------
# follow_line with improved recovery and debouncing
# ------------------------------------------------------------------
def follow_line():
    print_oled()
    oled.text("Follow line", 0, 40)
    oled.show()
    print("Follow line running")

    zero_count = 0
    MAX_ZERO_BEFORE_RECOVERY = 3  # Try small wiggle before giving up entirely

    while not check_collision():
        mid = read_ir_sensor(middle_IR)
        cl  = read_ir_sensor(center_left_IR)
        cr  = read_ir_sensor(center_right_IR)
        ol  = read_ir_sensor(outer_left_IR)
        or_ = read_ir_sensor(outer_right_IR)

        # Log sensor states for debugging (every few loops to avoid spam)
        if zero_count % 10 == 0:
            print(f"IR: mid={mid} cl={cl} cr={cr} ol={ol} or={or_}")

        # All zero -> line lost
        if mid == 0 and cl == 0 and cr == 0 and ol == 0 and or_ == 0:
            zero_count += 1
            if zero_count >= MAX_ZERO_BEFORE_RECOVERY:
                print("Line lost: attempting recovery wiggle")
                # Attempt small left-right search before exiting
                for _ in range(2):
                    motor_left.set_backwards()
                    motor_right.set_forwards()
                    motor_left.duty(insidewheel)
                    motor_right.duty(outsidewheel)
                    time.sleep(0.1)
                    stop()
                    time.sleep(0.05)
                    motor_left.set_forwards()
                    motor_right.set_backwards()
                    motor_left.duty(outsidewheel)
                    motor_right.duty(insidewheel)
                    time.sleep(0.1)
                    stop()
                    time.sleep(0.05)
                    # Check if line reacquired
                    if read_ir_sensor(middle_IR) or read_ir_sensor(center_left_IR) or read_ir_sensor(center_right_IR):
                        zero_count = 0
                        break
                if zero_count >= MAX_ZERO_BEFORE_RECOVERY:
                    # Still lost, exit and let no_line handle it
                    stop()
                    break
        else:
            zero_count = 0

        # Complex intersections: exit to process_sensors
        if (ol == 1 and mid == 1) or (or_ == 1 and mid == 1):
            stop()
            break

        # Centered: straight
        if mid == 1 and cl == 0 and cr == 0:
            motor_left.set_forwards()
            motor_right.set_forwards()
            motor_left.duty(slow)
            motor_right.duty(slow)
            print_oled()
            oled.text("Follow: Straight", 0, 40)
            oled.show()

        # Right deviation
        elif (cr == 1 and cl == 0 and mid == 0) or (or_ == 1 and cl == 0 and mid == 0) or (cr == 1 and cl == 0 and mid == 1):
            motor_left.set_forwards()
            motor_right.set_backwards()
            motor_left.duty(outsidewheel)
            motor_right.duty(insidewheel)
            print_oled()
            oled.text("Follow: Right", 0, 40)
            oled.show()

        # Left deviation
        elif (cl == 1 and cr == 0 and mid == 0) or (ol == 1 and cr == 0 and mid == 0) or (cl == 1 and cr == 0 and mid == 1):
            motor_left.set_backwards()
            motor_right.set_forwards()
            motor_left.duty(insidewheel)
            motor_right.duty(outsidewheel)
            print_oled()
            oled.text("Follow: Left", 0, 40)
            oled.show()

        # Recovery: only outer sensor sees line
        elif (ol == 1 and cl == 0 and cr == 0 and mid == 0 and or_ == 0) or (or_ == 1 and cl == 0 and cr == 0 and mid == 0 and ol == 0):
            if ol == 1:
                motor_left.set_backwards()
                motor_right.set_forwards()
                motor_left.duty(insidewheel)
                motor_right.duty(outsidewheel)
                print_oled()
                oled.text("Recover: Left", 0, 40)
                oled.show()
            elif or_ == 1:
                motor_left.set_forwards()
                motor_right.set_backwards()
                motor_left.duty(outsidewheel)
                motor_right.duty(insidewheel)
                print_oled()
                oled.text("Recover: Right", 0, 40)
                oled.show()
        else:
            # Unexpected pattern; exit and re-evaluate
            stop()
            break

        time.sleep(ontime)
        stop()
        time.sleep(offtime)

# ------------------------------------------------------------------
# handle_stub with correct direction
# ------------------------------------------------------------------
def handle_stub(side):
    print_oled()
    oled.text(f"handle stub: {side}", 0, 40)
    oled.show()
    print(f"Stub detected ({side})")
    if check_collision():
        return
    drive_forward()
    time.sleep(0.1)
    stop()

    mid = read_ir_sensor(middle_IR)
    cl  = read_ir_sensor(center_left_IR)
    cr  = read_ir_sensor(center_right_IR)

    if mid == 1 and cl == 0 and cr == 0:
        print("Stub cleared")
        return
    elif mid == 0 and cl == 0 and cr == 0:
        print("Most likely 90 degree turn.")
        if side == "LEFT":
            motor_left.set_backwards()
            motor_right.set_forwards()
        elif side == "RIGHT":
            motor_left.set_forwards()
            motor_right.set_backwards()
        else:
            stop()
            print("ERROR: invalid side in handle_stub")
            return
        motor_left.duty(slow)
        motor_right.duty(slow)
        time.sleep(0.2)
        stop()
    else:
        stop()
        print("ERROR: stub error, stopped.")
        return

# ------------------------------------------------------------------
# detect_y_intersection (unchanged but using debounced reads)
# ------------------------------------------------------------------
def detect_y_intersection(side):
    print_oled()
    oled.text(f"Y intersection: {side}", 0, 40)
    oled.show()
    stop()
    print(f"Y Intersection detected: taking {side} path.")
    time.sleep(0.1)

    if side == "LEFT":
        motor_left.set_backwards()
        motor_right.set_forwards()
        motor_left.duty(slow)
        motor_right.duty(slow)
        time.sleep(0.1)
        start = time.time()
        while True:
            if time.time() - start > pivottimeout:
                print("ERROR: Y-intersection LEFT pivot timed out")
                stop()
                return
            if read_ir_sensor(middle_IR) == 1 and read_ir_sensor(center_right_IR) == 0:
                break
            time.sleep(ontime)

    elif side == "RIGHT":
        motor_right.set_backwards()
        motor_left.set_forwards()
        motor_right.duty(slow)
        motor_left.duty(slow)
        time.sleep(0.1)
        start = time.time()
        while True:
            if time.time() - start > pivottimeout:
                print("ERROR: Y-intersection RIGHT pivot timed out")
                stop()
                return
            if read_ir_sensor(middle_IR) == 1 and read_ir_sensor(center_left_IR) == 0:
                break
            time.sleep(ontime)
    else:
        print(f"ERROR: detect_y_intersection: {side}, stopping.")
        stop()
        return

    time.sleep(0.05)
    motor_left.set_forwards()
    motor_right.set_forwards()
    motor_left.duty(slow)
    motor_right.duty(slow)

# ------------------------------------------------------------------
# no_line with timeout and corrected adjustment directions
# ------------------------------------------------------------------
def no_line():
    print_oled()
    oled.text("No line", 0,40)
    oled.show()
    print("No line: ultrasonic sensing")

    start_time = time.time()
    NO_LINE_TIMEOUT = 5.0

    while True:
        if check_collision():
            return
        if time.time() - start_time > NO_LINE_TIMEOUT:
            print("ERROR: no_line timed out, stopping")
            stop()
            return

        left_mm = ultrasonic_left.distance_mm()
        right_mm = ultrasonic_right.distance_mm()
        if left_mm <= 0 or right_mm <= 0:
            print("Ultrasonic error, skipping cycle")
            time.sleep(0.05)
            continue

        if right_mm == 0:
            right_mm = 1
        ratio = left_mm / right_mm

        # Center tolerance
        if (1 - centretollerance) <= ratio <= (1 + centretollerance):
            motor_left.set_forwards()
            motor_right.set_forwards()
            motor_left.duty(slow)
            motor_right.duty(slow)
        elif ratio < (1 - centretollerance):
            # Closer to left wall -> turn right
            motor_left.set_forwards()
            motor_right.set_backwards()
            motor_left.duty(slow)
            motor_right.duty(slow)
            print_oled()
            oled.text("No line: Turn right", 0,40)
            oled.show()
            time.sleep(0.05)
            stop()
            time.sleep(offtime)
            motor_left.set_forwards()
            motor_right.set_forwards()
            motor_left.duty(slow)
            motor_right.duty(slow)
        elif ratio > (1 + centretollerance):
            # Closer to right wall -> turn left
            motor_left.set_backwards()
            motor_right.set_forwards()
            motor_left.duty(slow)
            motor_right.duty(slow)
            print_oled()
            oled.text("No line: Turn left", 0,40)
            oled.show()
            time.sleep(0.05)
            stop()
            time.sleep(offtime)
            motor_left.set_forwards()
            motor_right.set_forwards()
            motor_left.duty(slow)
            motor_right.duty(slow)

        time.sleep(ontime)
        stop()
        time.sleep(offtime)

        # Check if line reappeared
        if (read_ir_sensor(middle_IR) or read_ir_sensor(outer_left_IR) or
            read_ir_sensor(outer_right_IR) or read_ir_sensor(center_left_IR) or
            read_ir_sensor(center_right_IR)):
            print("Line reacquired")
            return

# ------------------------------------------------------------------
# process_sensors with debounced reads
# ------------------------------------------------------------------
def process_sensors():
    stop()
    mid = read_ir_sensor(middle_IR)
    cl  = read_ir_sensor(center_left_IR)
    cr  = read_ir_sensor(center_right_IR)
    ol  = read_ir_sensor(outer_left_IR)
    or_ = read_ir_sensor(outer_right_IR)

    print(f"process_sensors: mid={mid} cl={cl} cr={cr} ol={ol} or={or_}")

    if cl == 1 and cr == 1 and mid == 1:
        roundabout()
    elif (cl == 1 and cr == 1 and mid == 0) or (ol == 1 and or_ == 1 and mid == 0):
        detect_y_intersection("LEFT")  # Default left; modify as needed
    elif ol == 1 and or_ == 0 and mid == 1:
        handle_stub("LEFT")
    elif or_ == 1 and ol == 0 and mid == 1:
        handle_stub("RIGHT")
    elif ol == 0 and or_ == 0 and cl == 0 and cr == 0 and mid == 0:
        no_line()
    elif (cl == 0 and cr == 0 and mid == 1) or (cl == 0 and mid == 0 and cr == 1) or (cl == 1 and mid == 0 and cr == 0):
        follow_line()
    else:
        print("ERROR: Unknown scenario, defaulting to follow_line")
        print_oled()
        oled.text("Unknown: Defaulting", 0,40)
        oled.show()
        follow_line()
    time.sleep(ontime)

# ------------------------------------------------------------------
# Roundabout Helper Functions (with debounced reads)
# ------------------------------------------------------------------
def turn_vehicle(direction):
    motor_left.set_forwards()
    motor_right.set_forwards()
    for i in range(11):
        motor_left.duty(slow)
        motor_right.duty(slow)
        time.sleep(ontime)
        motor_right.duty(0)
        motor_left.duty(0)
        time.sleep(offtime)
    if direction == 1:  # right
        motor_right.set_backwards()
        for i in range(10):
            motor_left.duty(slow)
            motor_right.duty(slow)
            time.sleep(ontime)
            motor_right.duty(0)
            motor_left.duty(0)
            time.sleep(offtime)
        while not read_ir_sensor(middle_IR):
            motor_left.duty(slow)
            motor_right.duty(slow)
            time.sleep(ontime)
            motor_right.duty(0)
            motor_left.duty(0)
            time.sleep(offtime)
    elif direction == 0:  # left
        motor_left.set_backwards()
        for i in range(10):
            motor_left.duty(slow)
            motor_right.duty(slow)
            time.sleep(ontime)
            motor_right.duty(0)
            motor_left.duty(0)
            time.sleep(offtime)
        while not read_ir_sensor(middle_IR):
            motor_left.duty(slow)
            motor_right.duty(slow)
            time.sleep(ontime)
            motor_right.duty(0)
            motor_left.duty(0)
            time.sleep(offtime)

def turn_in_roundabout(direction):
    motor_left.set_forwards()
    motor_right.set_forwards()
    for i in range(7):
        motor_left.duty(slow)
        motor_right.duty(slow)
        time.sleep(ontime)
        motor_right.duty(0)
        motor_left.duty(0)
        time.sleep(offtime)
    if direction == 1:  # right
        motor_right.set_backwards()
        while not read_ir_sensor(middle_IR):
            motor_left.duty(slow)
            motor_right.duty(slow)
            time.sleep(ontime)
            motor_right.duty(0)
            motor_left.duty(0)
            time.sleep(offtime)
    elif direction == 0:  # left
        motor_left.set_backwards()
        while not read_ir_sensor(middle_IR):
            motor_left.duty(slow)
            motor_right.duty(slow)
            time.sleep(ontime)
            motor_right.duty(0)
            motor_left.duty(0)
            time.sleep(offtime)

def turn_out_roundabout(direction):
    motor_left.set_forwards()
    motor_right.set_forwards()
    for i in range(8):
        motor_left.duty(slow)
        motor_right.duty(slow)
        time.sleep(ontime)
        motor_right.duty(0)
        motor_left.duty(0)
        time.sleep(offtime)
    turn_on_path(direction)

def turn_on_path(direction):
    if direction == 1:
        motor_right.set_backwards()
        while not read_ir_sensor(middle_IR):
            motor_left.duty(slow)
            motor_right.duty(slow)
            time.sleep(ontime)
            motor_right.duty(0)
            motor_left.duty(0)
            time.sleep(offtime)
    elif direction == 0:
        motor_left.set_backwards()
        while not read_ir_sensor(middle_IR):
            motor_left.duty(slow)
            motor_right.duty(slow)
            time.sleep(ontime)
            motor_right.duty(0)
            motor_left.duty(0)
            time.sleep(offtime)

# ------------------------------------------------------------------
# Roundabout Logic with corrected exit order
# ------------------------------------------------------------------
def roundabout():
    global direction_counter
    print_oled()
    oled.text("Roundabout", 0,40)
    oled.show()
    print("Roundabout detected")
    stop()
    direction = select_direction()
    turn_in_roundabout(direction)
    path = direction

    while True:
        follow_line()

        # Check if lost line completely
        if (read_ir_sensor(middle_IR) == 0 and read_ir_sensor(center_left_IR) == 0 and
            read_ir_sensor(center_right_IR) == 0 and read_ir_sensor(outer_left_IR) == 0 and
            read_ir_sensor(outer_right_IR) == 0):
            no_line()
            break

        # Exit detection
        if ((read_ir_sensor(outer_right_IR) == 1) or (read_ir_sensor(outer_left_IR) == 1) or
            (read_ir_sensor(center_right_IR) == 1 and read_ir_sensor(center_left_IR) == 1 and read_ir_sensor(middle_IR) == 1)):
            if read_ir_sensor(middle_IR) == 1:
                direction = select_direction()
                if direction == 1 or direction == 0:
                    turn_out_roundabout(direction)
                    break
                else:  # direction == 2 (straight)
                    skip_distraction_line()
                    break
            else:
                print_oled()
                oled.text("Roundabout: Exit w/o mid", 0,40)
                oled.show()
                if read_ir_sensor(outer_left_IR) == 1:
                    exit_dir = 0
                elif read_ir_sensor(outer_right_IR) == 1:
                    exit_dir = 1
                else:
                    exit_dir = 1
                turn_out_roundabout(exit_dir)
                break

def select_direction():
    global direction_counter
    # User requested: 1st exit = LEFT (0), 2nd = STRAIGHT (2), 3rd = RIGHT (1)
    # Mapping: 0=left, 1=right, 2=straight
    richtungen = [0, 2, 1]  # left, straight, right
    if direction_counter < len(richtungen):
        richtung = richtungen[direction_counter]
        direction_counter += 1
    else:
        direction_counter = 0
        richtung = richtungen[0]
    print(f"Roundabout direction selected: {richtung} (counter={direction_counter})")
    return richtung

def skip_distraction_line():
    motor_left.set_forwards()
    motor_right.set_forwards()
    for i in range(7):
        motor_left.duty(slow)
        motor_right.duty(slow)
        time.sleep(ontime)
        motor_right.duty(0)
        motor_left.duty(0)
        time.sleep(offtime)

# ------------------------------------------------------------------
# Start-up and Main Loop
# ------------------------------------------------------------------
motor_left.set_forwards()
motor_right.set_forwards()
print("Initialising systems... (1s)")
print_oled()
oled.text("Initialising systems... (1s)", 0,40)
oled.show()
time.sleep(1)
print("Running ✅")
print_oled()
oled.text("Running... ✅", 0,40)
oled.show()
time.sleep(1)
drive_forward()

while True:
    if not check_collision():
        process_sensors()
    else:
        stop()
        time.sleep(0.1)
