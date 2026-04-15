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
motor_left  = Motor("Left", 11, 10, 7)
motor_right = Motor("Right", 9, 8, 6)
ultrasonic_front = sonic(14, 15) 
ultrasonic_left  = sonic(3, 2)  
ultrasonic_right = sonic(12, 13)  

# Global counters
direction_counter = 0

# Setup important constants
slow = 40
outsidewheel = 45
insidewheel = 35
collisiondist = 50 # mm 
centretollerance = 0.4 # tolerance for left/right ratio for no_line
pivottimeout = 3.0 # sec
ontime = 0.02
offtime = 0.01

# OLED - I2C now uses GP4/GP5 which are free after fixing ultrasonic pins
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
    # FIX: replaced undefined left_IR/right_IR with correct sensor variables
    # Now displays all 5 IR sensors across two rows
    oled.fill(0)
    oled.text("OL:" + str(outer_left_IR.value()), 0, 10)
    oled.text("CL:" + str(center_left_IR.value()), 45, 10)
    oled.text("M:" + str(middle_IR.value()), 85, 10)
    oled.text("CR:" + str(center_right_IR.value()), 0, 20)
    oled.text("OR:" + str(outer_right_IR.value()), 45, 20)
    oled.show()

# Check front ultrasonic only — false if clear
def check_collision():
    # print_oled()
    # oled.text("check collision", 0, 40)
    # oled.show()
    front_mm = ultrasonic_front.distance_mm()
    if front_mm < 0: # error
        print("ERROR: negative ultrasonic dist, continuing")
        return False
    elif front_mm < collisiondist: # collision
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

def follow_line():
    print_oled()
    oled.text("follow line", 0, 40)
    oled.show()
    print("follow line running")

    MAX_ITERATIONS = 200  # Tune this to your loop timing
    iteration = 0

    while not check_collision():
        if iteration >= MAX_ITERATIONS:
            print("follow_line timeout — handing back to process_sensors")
            stop()
            break
        iteration += 1
        
        mid = middle_IR.value()
        cr  = center_right_IR.value()
        cl  = center_left_IR.value()
        or_ = outer_right_IR.value()
        ol  = outer_left_IR.value()

        
        # Centred: go straight
        if mid == 1 and cl == 0 and cr == 0 and ol == 0 and or_ == 0:
            motor_left.set_forwards()
            motor_right.set_forwards()
            motor_left.duty(slow)
            motor_right.duty(slow)

        # Minor RIGHT deviation: center_right sees line, pivot right gently
        elif (cr == 1 and cl == 0 and mid == 0 and or_ == 0 and ol == 0) or \
             (cr == 1 and cl == 0 and mid == 1 and or_ == 0 and ol == 0):
            motor_left.set_forwards()
            motor_right.set_backwards()
            motor_left.duty(outsidewheel)
            motor_right.duty(insidewheel)
            time.sleep(ontime)

        # Minor LEFT deviation: center_left sees line, pivot left gently
        elif (cl == 1 and cr == 0 and mid == 0 and or_ == 0 and ol == 0) or \
             (cl == 1 and cr == 0 and mid == 1 and or_ == 0 and ol == 0):
            motor_left.set_backwards()
            motor_right.set_forwards()
            motor_left.duty(insidewheel)
            motor_right.duty(outsidewheel)
            time.sleep(ontime)

        # Big RIGHT deviation: outer_right sees line, sharp pivot right
        elif (or_ == 1 and ol == 0 and cr == 0 and cl == 0 and mid == 0) or \
             (or_ == 1 and ol == 0 and cr == 1 and cl == 0 and mid == 0) or \
             (or_ == 1 and ol == 0 and cr == 1 and cl == 0 and mid == 1) or \
             (or_ == 1 and ol == 0 and cr == 0 and cl == 0 and mid == 1):
            motor_left.set_forwards()
            motor_right.set_backwards()
            motor_left.duty(fast)
            motor_right.duty(fast)
            time.sleep(ontime)

        # Big LEFT deviation: outer_left sees line, sharp pivot left
        elif (ol == 1 and or_ == 0 and cl == 0 and cr == 0 and mid == 0) or \
             (ol == 1 and or_ == 0 and cl == 1 and cr == 0 and mid == 0) or \
             (ol == 1 and or_ == 0 and cl == 1 and cr == 0 and mid == 1) or \
             (ol == 1 and or_ == 0 and cl == 0 and cr == 0 and mid == 1):
            motor_left.set_backwards()
            motor_right.set_forwards()
            motor_left.duty(fast)
            motor_right.duty(fast)
            time.sleep(ontime)

        # Y-intersection, roundabout, or no line, exit and let process_sensors handle it
        else:
            stop()
            break

        # Stubs/intersections: exit and let process_sensors handle it
        if ol == 1 and or_ == 1:
            stop()
            break

        time.sleep(ontime)
        stop()
        time.sleep(offtime)

def handle_stub(side):
    print_oled()
    oled.text("handle stub", 0, 40)
    oled.show()
    print(f"Stub detected ({side})")
    if check_collision():
        return
    drive_forward()
    time.sleep(0.1) # actual time req UNKNOWN — NEED TO TEST
    stop()
    if middle_IR.value() == 1 and center_left_IR.value() == 0 and center_right_IR.value() == 0:
        print("Stub cleared")
        return
    elif middle_IR.value() == 0 and center_left_IR.value() == 0 and center_right_IR.value() == 0:
        print("Most likely 90 degree turn.")
    else:
        stop()
        print("ERROR: stub error, stopped.")
        return

def detect_y_intersection(side):
    print_oled()
    oled.text("y intersection", 0, 40)
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
            if middle_IR.value() == 1 and center_right_IR.value() == 0:
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
            if middle_IR.value() == 1 and center_left_IR.value() == 0:
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

def no_line():
    print_oled()
    oled.text("no line", 0, 40)
    oled.show()
    print("No line: ultrasonic sensing")

    while True:
        if check_collision():
            return

        left_mm = ultrasonic_left.distance_mm()
        right_mm = ultrasonic_right.distance_mm()
        if left_mm <= 0 or right_mm <= 0:
            print("Ultrasonic error, skipping cycle")
            continue

        ratio = left_mm / right_mm
        if (1 - centretollerance) <= ratio <= (1 + centretollerance):
            motor_left.set_forwards()
            motor_right.set_forwards()
            motor_left.duty(slow)
            motor_right.duty(slow)
        elif ratio < (1 - centretollerance):
            motor_right.set_backwards()
            motor_left.set_forwards()
            motor_left.duty(slow)
            motor_right.duty(slow)
        elif ratio > (1 + centretollerance):
            motor_right.set_forwards()
            motor_left.set_backwards()
            motor_left.duty(slow)
            motor_right.duty(slow)

        time.sleep(ontime)
        stop()
        time.sleep(offtime)

        if middle_IR.value() == 1 or outer_left_IR.value() == 1 or outer_right_IR.value() == 1:
            print("Line reacquired")
            return
    
def stop():
   # print_oled()
   # oled.text("stop", 0, 40)
   # oled.show()
    motor_left.duty(0)
    motor_right.duty(0)
    
# Detection
def process_sensors():
    stop()
    if center_left_IR.value() == 1 and center_right_IR.value() == 1 and middle_IR.value() == 1:
        roundabout()
    elif (center_left_IR.value() == 1 and center_right_IR.value() == 1 and middle_IR.value() == 0) or (outer_left_IR.value() == 1 and outer_right_IR.value() == 1 and middle_IR.value() == 0):
        # FIX: use select_direction() instead of hardcoding "LEFT"
        direction = select_direction()
        if direction == 1:
            detect_y_intersection("RIGHT")
        elif direction == 0:
            detect_y_intersection("LEFT")
        else:
            # direction == 2: go straight through the intersection
            drive_forward()
            time.sleep(ontime)
            stop()
    elif outer_left_IR.value() == 1 and outer_right_IR.value() == 0 and middle_IR.value() == 1:
        handle_stub("LEFT")
    elif outer_right_IR.value() == 1 and outer_left_IR.value() == 0 and middle_IR.value() == 1:
        handle_stub("RIGHT")
    elif outer_left_IR.value() == 0 and outer_right_IR.value() == 0 and center_left_IR.value() == 0 and center_right_IR.value() == 0 and middle_IR.value() == 0:
        no_line()
    elif (center_left_IR.value() == 0 and center_right_IR.value() == 0 and middle_IR.value() == 1) or (center_left_IR.value() == 0 and middle_IR.value() == 0 and center_right_IR.value() == 1) or (center_left_IR.value() == 0 and middle_IR.value() == 1 and center_right_IR.value() == 1) or (center_left_IR.value() == 1 and middle_IR.value() == 0 and center_right_IR.value() == 0) or (center_left_IR.value() == 1 and middle_IR.value() == 1 and center_right_IR.value() == 0):
        follow_line()
    else:
        print("ERROR: Unknown scenario.")
        print_oled()
        oled.text("UNKNOW: Defaulting", 0, 40)
        oled.show()
        follow_line()
    time.sleep(ontime)

def turn_vehicle(direction):
    motor_left.set_forwards()
    motor_right.set_forwards()
    for i in range(11):
        motor_left.duty(50)
        motor_right.duty(50)
        time.sleep(ontime)
        motor_right.duty(0)
        motor_left.duty(0)
        time.sleep(offtime)
        
    if direction == 1: # right
        motor_right.set_backwards()
        for i in range(10):
            motor_left.duty(50)
            motor_right.duty(50)
            time.sleep(ontime)
            motor_right.duty(0)
            motor_left.duty(0)
            time.sleep(offtime)
        while not middle_IR.value():
            motor_left.duty(slow)
            motor_right.duty(slow)
            time.sleep(ontime)
            motor_right.duty(0)
            motor_left.duty(0)
            time.sleep(offtime)
    elif direction == 0: # left
        motor_left.set_backwards()
        for i in range(10):
            motor_left.duty(50)
            motor_right.duty(50)
            time.sleep(ontime)
            motor_right.duty(0)
            motor_left.duty(0)
            time.sleep(offtime)
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
        time.sleep(ontime)
        motor_right.duty(0)
        motor_left.duty(0)
        time.sleep(offtime)
        
    if direction == 1: # right
        motor_right.set_backwards()
        while not middle_IR.value():
            motor_left.duty(slow)
            motor_right.duty(slow)
            time.sleep(ontime)
            motor_right.duty(0)
            motor_left.duty(0)
            time.sleep(offtime)
    elif direction == 0: # left
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
        time.sleep(ontime)
        motor_right.duty(0)
        motor_left.duty(0)
        time.sleep(offtime)
        
    turn_on_path(direction)

def turn_on_path(direction):
    if direction == 1: # right
        motor_right.set_backwards()
        while not middle_IR.value():
            motor_left.duty(slow)
            motor_right.duty(slow)
            time.sleep(ontime)
            motor_right.duty(0)
            motor_left.duty(0)
            time.sleep(offtime)
    elif direction == 0: # left
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

def roundabout():
    print_oled()
    oled.text("roundabout", 0, 40)
    oled.show()
    print("Roundabout detected")
    stop()
    direction = select_direction()
    turn_in_roundabout(direction)
    path = direction
    
    while True:
        follow_line()
        if ((outer_right_IR.value() == 1) or (outer_left_IR.value() == 1) or (center_right_IR.value() == 1 and center_left_IR.value() == 1 and middle_IR.value() == 1)):
            if (middle_IR.value() == 1):
                direction = select_direction()
                if direction == 1 or direction == 0:
                    turn_out_roundabout(direction)
                    break  # exit roundabout
                else:
                    # FIX: direction == 2 (straight) now breaks after skipping
                    # to prevent infinite loop when select_direction returns 2
                    skip_distraction_line()
                    if path == 1:
                        turn_on_path(0)
                    else:
                        turn_on_path(1)
                    break
            else:
                # fault handling
                pass

def select_direction():
    # 0 = left, 1 = right, 2 = straight
    richtungen = [1, 1, 2, 1] # for every intersection in roundabout one direction
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
        time.sleep(ontime)
        motor_right.duty(0)
        motor_left.duty(0)
        time.sleep(offtime)
    
# Start-up
motor_left.set_forwards()
motor_right.set_forwards()
print("Starting in 2 seconds...")
print_oled()
oled.text("Starting in 2 seconds...", 0, 40)
oled.show()
#time.sleep(2)
print("Running")
print_oled()
oled.text("Running", 0, 40)
oled.show()
drive_forward()

# Main loop
while True:        
    if not check_collision():
        process_sensors()
    else:
        stop()
        time.sleep(0.1)
