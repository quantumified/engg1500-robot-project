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
motor_left  = Motor("Left", 11, 10, 7) #probably need to find way to make them slower without duty reduction, maybe make a pwm of a pwm??
motor_right = Motor("Right", 9, 8, 6)
ultrasonic_front = sonic(3, 2)
ultrasonic_left  = sonic(5, 4) #same for these
ultrasonic_right = sonic(17, 16)

#global counters
direction_counter = 0

# Setup important constants
slow = 40
outsidewheel = 45
insidewheel = 35
collisiondist = 50 # mm 
centretollerance = 0.2 # tolerance for left/right ratio for no_line
pivottimeout = 3.0 # sec
ontime = 0.02
offtime = 0.01

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
    oled.text(str(left_IR.value()), 0, 10)
    oled.text(str(middle_IR.value()), 50, 10)
    oled.text(str(right_IR.value()), 90, 10)
    oled.show()

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

def follow_line():
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

def handle_stub(side):
    print_oled()
    oled.text("handle stub", 0, 40)
    oled.show()
    print(f"Stub detected ({side})")
    if check_collision(): # True = collision
        return
    drive_forward()
    time.sleep(0.1) #actual time req UNKNOWN NEED TO TEST!
    stop()
    if middle_IR.value() == 1 and center_left_IR.value() == 0 and center_right_IR.value() == 0:
        print("Stub cleared")
        return
        # main loop takes over
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
        # Pivot left on the spot
        motor_left.set_backwards()
        motor_right.set_forwards()
        motor_left.duty(slow)
        motor_right.duty(slow)
        time.sleep(0.1) #actual move time to get to left path for eg. needs to be TESTED and implemented
        start = time.time()

        while True:
            if time.time() - start > pivottimeout:
                print("ERROR: Y-intersection LEFT pivot timed out")
                stop()
                return
            if middle_IR.value() == 1 and center_right_IR.value() == 0:
                break
            time.sleep(0.01)

    elif side == "RIGHT":
        # Pivot right on the spot
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
            time.sleep(0.01)

    else:
        print(f"ERROR: detect_y_intersection: {side}, stopping.")
        stop()
        return
    time.sleep(0.05)

    # Resume line following
    motor_left.set_forwards()
    motor_right.set_forwards()
    motor_left.duty(slow)
    motor_right.duty(slow)

def no_line():
    # Announcements
    print_oled()
    oled.text("no line", 0, 40)
    oled.show()
    print("No line: ultrasonic sensing")

    while True:
        if check_collision():
            return

        left_mm = ultrasonic_left.distance_mm()
        right_mm = ultrasonic_right.distance_mm()
        # Check no issues
        if left_mm <= 0 or right_mm <= 0:
            print("Ultrasonic error, skipping cycle")
            continue

        # Ratio (1.0 = perfectly centered)
        ratio = left_mm / right_mm
        # Within tolerance, then continue
        if (1 - centretollerance) <= ratio <= (1 + centretollerance):
            motor_left.set_forwards()
            motor_right.set_forwards()
            motor_left.duty(slow)
            motor_right.duty(slow)

        # Steer right
        elif ratio < (1 - centretollerance):
            motor_left.set_backwards()
            motor_right.set_forwards()
            motor_left.duty(slow) 
            motor_right.duty(slow)  

        # Steer left
        elif ratio > (1 + centretollerance):
            motor_left.set_forwards()
            motor_right.set_backwards()
            motor_left.duty(slow)
            motor_right.duty(slow)

        time.sleep(ontime)
        stop()
        time.sleep(offtime)

        # Cancel if line is found again
        if middle_IR.value() == 1 or left_IR.value() == 1 or right_IR.value() == 1:
            print("Line reacquired")
            return
    
def stop():
    print_oled()
    oled.text("stop", 0, 40)
    oled.show()
    motor_left.duty(0)
    motor_right.duty(0)
    
# Detection
def process_sensors():
    stop()
    if center_left_IR.value() == 1 and center_right_IR.value() == 1 and middle_IR.value() == 1: #handle roundabout
        roundabout()
    elif (center_left_IR.value() == 1 and center_right_IR.value() == 1 and middle_IR.value() == 0) or (outer_left_IR.value() == 1 and outer_right_IR.value() == 1 and middle_IR.value() == 0): # This logic for detecting y intersections needs to be fixed (I don't think it works due to speed of motors being too high and either skipping or following random path instead of specified path due to this logic not even being triggered)
        detect_y_intersection("LEFT")  # change to "RIGHT" to take the other branch
    elif outer_left_IR.value() == 1 and outer_right_IR.value() == 0 and middle_IR.value() == 1:
        handle_stub("LEFT")
    elif outer_right_IR.value() == 1 and outer_left_IR.value() == 0 and middle_IR.value() == 1:
        handle_stub("RIGHT")
    elif outer_left_IR.value() == 0 and outer_right_IR.value() == 0 and center_left_IR.value() == 0 and center_right_IR.value() == 0 and middle_IR.value() == 0:
        no_line()
    elif (center_left_IR.value() == 0 and center_right_IR.value() == 0 and middle_IR.value() == 1) or (center_left_IR.value() == 0 and middle_IR.value() == 0 and center_right_IR.value() == 1) or (center_left_IR.value() == 0 and middle_IR.value() == 1 and center_right_IR.value() == 1) or (center_left_IR.value() == 1 and middle_IR.value() == 0 and center_right_IR.value() == 0) or (center_left_IR.value() == 1 and middle_IR.value() == 1 and center_right_IR.value() == 0): # Go straight
        follow_line()
    else:
        print("ERROR: Unknown senario.")
        follow_line()
    time.sleep(0.02)

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
        for i in range(10):
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
    elif direction == 0: #left
        motor_left.set_backwards()
        for i in range(10):
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

def select_direction():
    #0 = left, 1 = right, 2 = straight
    richtungen = [1, 1, 2, 1]#for every intersection in roundabout one direction
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
    
# Start-up
motor_left.set_forwards()
motor_right.set_forwards()
print("Starting in 2 seconds...")
#time.sleep(2)
print("Running")
drive_forward()

# Main loop
while True:        
    if not check_collision(): # if false runs this
        process_sensors() # need to ensure follow_line is disabled/cancelled when another function is triggered ASAP!
    else:
        stop()
        time.sleep(0.1)
