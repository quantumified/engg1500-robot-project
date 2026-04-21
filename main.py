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
motor_left  = Motor("left", 11, 10, 7) #probably need to find way to make them slower without duty reduction, maybe make a pwm of a pwm??
motor_right = Motor("right", 9, 8, 6)
ultrasonic_front = sonic(14, 15)
ultrasonic_left  = sonic(3, 2) #same for these
ultrasonic_right = sonic(12, 13)

#global counters
direction_counter = 0

# Setup important constants
slow = 47
outsidewheel = 50
outsidewheel_left = 55
insidewheel = 40
outsidewheel_slow = 45
insidewheel_slow = 40
collisiondist = 50 # mm 
centretollerance = 0.2 # tolerance for left/right ratio for no_line
pivottimeout = 3.0 # sec
ontime = 0.03
offtime = 0.1

# OLED bullshit
#You can choose any other combination of I2C pins
i2c = SoftI2C(scl=Pin(5), sda=Pin(4))

# Creates an object for the display using the class in the ssd1306 driver
oled_width = 128
oled_height = 64
oled = ssd1306.SSD1306_I2C(oled_width, oled_height, i2c)

oled.text('Left', 0, 0)
oled.text('Mid', 50, 0)
oled.text('Right', 90, 0)
oled.text("Current Function", 0, 30)
oled.invert(True)
oled.show()

def print_oled():
    oled.fill(0)
    oled.text(str(center_left_IR.value()), 0, 10)
    oled.text(str(middle_IR.value()), 50, 10)
    oled.text(str(center_right_IR.value()), 90, 10)
    oled.show()
    
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

# Check front ultrasonic only false if clear
def check_collision():
    print_oled()
    oled.text("check collision", 0, 40)
    oled.show()
    front_mm = ultrasonic_front.distance_mm()
    if front_mm < 0: #error
        print("ERROR: negative ultrasonic dist, continuing")
        return False
    elif front_mm < collisiondist: #collision
        print(f"Collision: {front_mm} mm room left")
        stop()
        return True
    return False

# Motor actions
def drive_forward():
    motor_left.set_forwards()
    motor_right.set_forwards()
    motor_left.duty(slow)
    motor_right.duty(slow)

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
            pass
            #print(f"IR: mid={mid} cl={cl} cr={cr} ol={ol} or={or_}")

        # All zero -> line lost
        if mid == 0 and cl == 0 and cr == 0 and ol == 0 and or_ == 0:
            no_line()
            break
           
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
        elif (cr == 1 and cl == 0 and mid == 0) or (or_ == 1 and cl == 0 and mid == 0):
            motor_left.set_forwards()
            motor_right.set_forwards()
            motor_left.duty(outsidewheel_left)
            motor_right.duty(insidewheel)
            print_oled()
            oled.text("Follow: Right", 0, 40)
            oled.show()
        elif (cr == 1 and cl == 0 and mid == 1):
            motor_left.set_forwards()
            motor_right.set_forwards()
            motor_left.duty(outsidewheel)
            motor_right.duty(insidewheel_slow)
            print_oled()
            oled.text("Follow: Right", 0, 40)
            oled.show()

        # Left deviation
        elif (cl == 1 and cr == 0 and mid == 0) or (ol == 1 and cr == 0 and mid == 0):
            motor_left.set_forwards()
            motor_right.set_forwards()
            motor_left.duty(insidewheel)
            motor_right.duty(outsidewheel)
            print_oled()
            oled.text("Follow: Left", 0, 40)
            oled.show()
        elif (cl == 1 and cr == 0 and mid == 1):
            motor_left.set_forwards()
            motor_right.set_forwards()
            motor_left.duty(insidewheel_slow)
            motor_right.duty(outsidewheel_slow)
            print_oled()
            oled.text("Follow: Left", 0, 40)
            oled.show()

        # Recovery: only outer sensor sees line
        elif (ol == 1 and cl == 0 and cr == 0 and mid == 0 and or_ == 0) or (or_ == 1 and cl == 0 and cr == 0 and mid == 0 and ol == 0):
            if ol == 1:
                motor_left.set_forwards()
                motor_right.set_forwards()
                motor_left.duty(insidewheel)
                motor_right.duty(outsidewheel)
                print_oled()
                oled.text("Recover: Left", 0, 40)
                oled.show()
            elif or_ == 1:
                motor_left.set_forwards()
                motor_right.set_forwards()
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
        motor_left.set_backwards()
        motor_right.set_backwards()
        motor_left.duty(slow)
        motor_right.duty(slow)
        time.sleep(0.1)
        stop()
        if side == "LEFT":
            turn_vehicle(0)
        elif side == "RIGHT":
            turn_vehicle(1)
        else:
            stop()
            print("ERROR: invalid side in handle_stub")
            return
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
    
    if (ultrasonic_left.distance_mm() > 150) and (ultrasonic_right.distance_mm() > 150):
        print("just no line")
        dont_turn_to_other_side = False
        
        turnvar = 8 # how fast does the vehicle turn
        motor_left.set_backwards()
        motor_right.set_forwards()
        for i in range(turnvar):
            motor_left.duty(slow)
            motor_right.duty(slow)
            time.sleep(0.02)
            stop()
            time.sleep(0.1)
            if read_ir_sensor(middle_IR) == 1:
                for i in range(4):
                    motor_left.duty(slow)
                    motor_right.duty(slow)
                    time.sleep(0.02)
                    stop()
                    time.sleep(0.1)
                dont_turn_to_other_side = True
                break
        if (ultrasonic_left.distance_mm() < 100) or (ultrasonic_right.distance_mm() < 100):
            motor_left.set_forwards()
            motor_right.set_forwards()
            for i in range(10):
                motor_left.duty(slow)
                motor_right.duty(slow)
                time.sleep(0.02)
                stop()
                time.sleep(0.1)
            return
        for i in range(turnvar):
            motor_left.duty(slow)
            motor_right.duty(slow)
            time.sleep(0.02)
            stop()
            time.sleep(0.1)
            if read_ir_sensor(middle_IR) == 1:
                for i in range(4):
                    motor_left.duty(slow)
                    motor_right.duty(slow)
                    time.sleep(0.02)
                    stop()
                    time.sleep(0.1)
                dont_turn_to_other_side = True
                break
        if (ultrasonic_left.distance_mm() < 100) or (ultrasonic_right.distance_mm() < 100):
            motor_left.set_forwards()
            motor_right.set_forwards()
            for i in range(10):
                motor_left.duty(slow)
                motor_right.duty(slow)
                time.sleep(0.02)
                stop()
                time.sleep(0.1)
            return
        if (dont_turn_to_other_side == False):
            motor_left.set_forwards()
            motor_right.set_backwards()
            for i in range(turnvar * 2):
                motor_left.duty(slow)
                motor_right.duty(slow)
                time.sleep(0.02)
                stop()
                time.sleep(0.1)
                if read_ir_sensor(middle_IR) == 1:
                    for i in range(4):
                        motor_left.duty(slow)
                        motor_right.duty(slow)
                        time.sleep(0.02)
                        stop()
                        time.sleep(0.1)
                    return
            if (ultrasonic_left.distance_mm() < 100) or (ultrasonic_right.distance_mm() < 100):
                motor_left.set_forwards()
                motor_right.set_forwards()
                for i in range(10):
                    motor_left.duty(slow)
                    motor_right.duty(slow)
                    time.sleep(0.02)
                    stop()
                    time.sleep(0.1)
                return
            for i in range(turnvar * 2):
                motor_left.duty(slow)
                motor_right.duty(slow)
                time.sleep(0.02)
                stop()
                time.sleep(0.1)
                if read_ir_sensor(middle_IR) == 1:
                    for i in range(4):
                        motor_left.duty(slow)
                        motor_right.duty(slow)
                        time.sleep(0.02)
                        stop()
                        time.sleep(0.1)
                    return
        return              

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
        if (left_mm > 150) and (right_mm > 150):
            print ("no line after hallway")
            motor_left.set_forwards()
            motor_right.set_forwards()
            motor_left.duty(slow)
            motor_right.duty(slow)
        elif (1 - centretollerance) <= ratio <= (1 + centretollerance):
            print("hallway ride")
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

        hallway_ontime = ontime * 3
        time.sleep(hallway_ontime)
        stop()

        # Check if line reappeared
        if (read_ir_sensor(middle_IR) or read_ir_sensor(outer_left_IR) or
            read_ir_sensor(outer_right_IR) or read_ir_sensor(center_left_IR) or
            read_ir_sensor(center_right_IR)):
            print("Line reacquired")
            return
    
def turn_left_first:
    turnvar = 8 # how fast does the vehicle turn
        motor_left.set_backwards()
        motor_right.set_forwards()
        for i in range(turnvar):
            motor_left.duty(slow)
            motor_right.duty(slow)
            time.sleep(0.02)
            stop()
            time.sleep(0.1)
            if read_ir_sensor(middle_IR) == 1:
                for i in range(4):
                    motor_left.duty(slow)
                    motor_right.duty(slow)
                    time.sleep(0.02)
                    stop()
                    time.sleep(0.1)
                dont_turn_to_other_side = True
                break
        if (ultrasonic_left.distance_mm() < 100) or (ultrasonic_right.distance_mm() < 100):
            motor_left.set_forwards()
            motor_right.set_forwards()
            for i in range(10):
                motor_left.duty(slow)
                motor_right.duty(slow)
                time.sleep(0.02)
                stop()
                time.sleep(0.1)
            return
        for i in range(turnvar):
            motor_left.duty(slow)
            motor_right.duty(slow)
            time.sleep(0.02)
            stop()
            time.sleep(0.1)
            if read_ir_sensor(middle_IR) == 1:
                for i in range(4):
                    motor_left.duty(slow)
                    motor_right.duty(slow)
                    time.sleep(0.02)
                    stop()
                    time.sleep(0.1)
                dont_turn_to_other_side = True
                break
        if (ultrasonic_left.distance_mm() < 100) or (ultrasonic_right.distance_mm() < 100):
            motor_left.set_forwards()
            motor_right.set_forwards()
            for i in range(10):
                motor_left.duty(slow)
                motor_right.duty(slow)
                time.sleep(0.02)
                stop()
                time.sleep(0.1)
            return
        if (dont_turn_to_other_side == False):
            motor_left.set_forwards()
            motor_right.set_backwards()
            for i in range(turnvar * 2):
                motor_left.duty(slow)
                motor_right.duty(slow)
                time.sleep(0.02)
                stop()
                time.sleep(0.1)
                if read_ir_sensor(middle_IR) == 1:
                    for i in range(4):
                        motor_left.duty(slow)
                        motor_right.duty(slow)
                        time.sleep(0.02)
                        stop()
                        time.sleep(0.1)
                    return
            if (ultrasonic_left.distance_mm() < 100) or (ultrasonic_right.distance_mm() < 100):
                motor_left.set_forwards()
                motor_right.set_forwards()
                for i in range(10):
                    motor_left.duty(slow)
                    motor_right.duty(slow)
                    time.sleep(0.02)
                    stop()
                    time.sleep(0.1)
                return
            for i in range(turnvar * 2):
                motor_left.duty(slow)
                motor_right.duty(slow)
                time.sleep(0.02)
                stop()
                time.sleep(0.1)
                if read_ir_sensor(middle_IR) == 1:
                    for i in range(4):
                        motor_left.duty(slow)
                        motor_right.duty(slow)
                        time.sleep(0.02)
                        stop()
                        time.sleep(0.1)
                    return
        return    

def stop():
    motor_left.duty(0)
    motor_right.duty(0)
    
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

    #print(f"process_sensors: mid={mid} cl={cl} cr={cr} ol={ol} or={or_}")

    if (cl == 1 and cr == 1 and mid == 1) or (cl == 1 and cr == 1 and mid == 0) or (ol == 1 and or_ == 1 and mid == 0):
        roundabout() #y-intesection is also handled with the roundabout method
    elif ol == 1 and or_ == 0 and mid == 1:
        handle_stub("LEFT")
    elif or_ == 1 and ol == 0 and mid == 1:
        handle_stub("RIGHT")
    elif cl == 0 and cr == 0 and mid == 0:
        no_line()
    elif (cl == 0 and cr == 0 and mid == 1) or (cl == 0 and mid == 0 and cr == 1) or (cl == 1 and mid == 0 and cr == 0):
        follow_line()
    else:
        print("ERROR: Unknown scenario, defaulting to follow_line")
        print_oled()
        oled.text("Unknown: Defaulting", 0,40)
        oled.show()
        motor_left.set_forwards()
        motor_right.set_forwards()
        for i in range(8):
            motor_left.duty(slow)
            motor_right.duty(slow)
            time.sleep(0.02)
            stop()
        follow_line()
    time.sleep(ontime)

def turn_vehicle(direction):
    motor_left.set_forwards()
    motor_right.set_forwards()
    for i in range(11):
        motor_left.duty(50)
        motor_right.duty(50)
        time.sleep(0.02)
        motor_right.duty(0)
        motor_left.duty(0)
        time.sleep(0.03)
        
    if direction == 1: #right
        motor_right.set_backwards()
        for i in range(5):
            motor_left.duty(50)
            motor_right.duty(50)
            time.sleep(0.02)
            motor_right.duty(0)
            motor_left.duty(0)
            time.sleep(0.02)
        while not middle_IR.value():
            motor_left.duty(50)
            motor_right.duty(50)
            time.sleep(ontime)
            motor_right.duty(0)
            motor_left.duty(0)
            time.sleep(offtime)
    elif direction == 0: #left
        motor_left.set_backwards()
        for i in range(5):
            motor_left.duty(50)
            motor_right.duty(50)
            time.sleep(0.02)
            motor_right.duty(0)
            motor_left.duty(0)
            time.sleep(0.02)
        while not middle_IR.value():
            motor_left.duty(slow)
            motor_right.duty(slow)
            time.sleep(ontime)
            motor_right.duty(0)
            motor_left.duty(0)
            time.sleep(offtime)
    else:
        pass
    
def turn_in_roundabout(direction):
    motor_left.set_forwards()
    motor_right.set_forwards()
    for i in range(7):
        motor_left.duty(50)
        motor_right.duty(50)
        time.sleep(0.02)
        motor_right.duty(0)
        motor_left.duty(0)
        time.sleep(0.03)
        
    if direction == 1: #right
        motor_right.set_backwards()
        while not middle_IR.value():
            motor_left.duty(slow)
            motor_right.duty(slow)
            time.sleep(ontime)
            motor_right.duty(0)
            motor_left.duty(0)
            time.sleep(offtime)
    elif direction == 0: #left
        motor_left.set_backwards()
        while not middle_IR.value():
            motor_left.duty(slow)
            motor_right.duty(slow)
            time.sleep(ontime)
            motor_right.duty(0)
            motor_left.duty(0)
            time.sleep(offtime)
    else:
        pass
    
def turn_out_roundabout(direction):
    motor_left.set_forwards()
    motor_right.set_forwards()
    for i in range(8):
        motor_left.duty(50)
        motor_right.duty(50)
        time.sleep(0.02)
        motor_right.duty(0)
        motor_left.duty(0)
        time.sleep(0.02)
        
    turn_on_path(direction)

def turn_on_path(direction):
    if direction == 1: #right
        motor_right.set_backwards()
        while not middle_IR.value():
            motor_left.duty(slow)
            motor_right.duty(slow)
            time.sleep(0.02)
            motor_right.duty(0)
            motor_left.duty(0)
            time.sleep(0.02)
    elif direction == 0: #left
        motor_left.set_backwards()
        while not middle_IR.value():
            motor_left.duty(slow)
            motor_right.duty(slow)
            time.sleep(0.02)
            motor_right.duty(0)
            motor_left.duty(0)
            time.sleep(0.02)
    else:
        pass

def roundabout():
    print_oled()
    oled.text("roundabout", 0, 40)
    oled.show()
    print("Roundabout detected")
    stop()
    direction = select_direction()
    turn_in_roundabout(direction)
    path = direction #in which direction the vehicle drives
    
    while True:
        follow_line()
        if ((outer_right_IR.value() == 1) or (outer_left_IR.value() == 1) or (center_right_IR.value() == 1 and center_left_IR.value() == 1 and middle_IR.value() == 1)):
            if (middle_IR.value() == 1): #helpful?????
                direction = select_direction()
                if ((direction == 1) or (direction == 0)): #1:right, 0:left
                    turn_out_roundabout(direction)
                    break
                else:
                    skip_distraction_line()
                    if (path == 1):
                        turn_on_path(0)
                    else:
                        turn_on_path(1)
            else:
                #faulthandling
                pass
            
def follow_line_roundi():
    print_oled()
    oled.text("follow line", 0, 40)
    oled.show()
    print("follow line running")
    while not check_collision():
        # Centred: go straight 
        if middle_IR.value() == 1 and center_left_IR.value() == 0 and center_right_IR.value() == 0:
            motor_left.set_forwards()
            motor_right.set_forwards()
            motor_left.duty(slow)
            motor_right.duty(slow)

        # RIGHT deviation: pivot right
        elif (center_right_IR.value() == 1 and center_left_IR.value() == 0 and middle_IR.value() == 0) or (center_right_IR.value() == 1 and center_left_IR.value() == 0 and middle_IR.value() == 1):
            motor_left.set_forwards()
            motor_right.set_backwards()
            motor_left.duty(outsidewheel)
            motor_right.duty(insidewheel)
            time.sleep(0.02)

        # LEFT deviation: pivot left
        elif (center_left_IR.value() == 1 and center_right_IR.value() == 0 and middle_IR.value() == 0) or (center_left_IR.value() == 1 and center_right_IR.value() == 0 and middle_IR.value() == 1): 
            motor_left.set_backwards()
            motor_right.set_forwards()
            motor_left.duty(insidewheel)
            motor_right.duty(outsidewheel)
            time.sleep(0.02)
            
        #Y-intersection, roundabout, no line
        else:
            stop()
            break
        
        #stubs/intersections
        if outer_left_IR.value() == 1 or outer_right_IR.value() == 1:
            stop()
            break
        
        time.sleep(ontime)
        stop()
        time.sleep(offtime)

def select_direction():
    #0 = left, 1 = right, 2 = straight
    richtungen = [0, 0, 0, 2, 0, 1, 1]#for every intersection in roundabout one direction
    global direction_counter
    
    if direction_counter < len(richtungen):
        richtung = richtungen[direction_counter]
        direction_counter += 1
        return richtung
    else:
        direction_counter = 0
        return 1
        
def skip_distraction_line():
    motor_left.set_forwards()
    motor_right.set_forwards()
    for i in range(7):
        motor_left.duty(50)
        motor_right.duty(50)
        time.sleep(0.02)
        motor_right.duty(0)
        motor_left.duty(0)
        time.sleep(0.02)
        
def init():
    right_distance = 0
    left_distance = 0
    print("init runs")
    while middle_IR.value() == 0 and center_left_IR.value() == 0 and center_right_IR.value() == 0:
        motor_left.set_forwards()
        motor_right.set_forwards()
        motor_left.duty(slow)
        motor_right.duty(slow)
        time.sleep(ontime)
        stop()
        time.sleep(offtime)
    print("hallway durch")
    follow_line()
    print("vorwärts fahrt durch")
    print("init done")
    
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

# Main loop
while True:        
    if not check_collision(): # if false runs this
        process_sensors() # need to ensure follow_line is disabled/cancelled when another function is triggered ASAP!
    else:
        stop()
        time.sleep(0.1)
