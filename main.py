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

# Initialise OLED
i2c = SoftI2C(scl=Pin(5), sda=Pin(4))
oled_width = 128
oled_height = 64
oled = ssd1306.SSD1306_I2C(oled_width, oled_height, i2c)
# Setup 
oled.invert(True) # Background to white, text black
def print_oled():
    oled.fill(0)
    oled.text("OL:" + str(outer_left_IR.value()), 0, 10)
    oled.text("CL:" + str(center_left_IR.value()), 45, 10)
    oled.text("M:" + str(middle_IR.value()), 85, 10)
    oled.text("CR:" + str(center_right_IR.value()), 0, 20)
    oled.text("OR:" + str(outer_right_IR.value()), 45, 20)
    oled.show()

# Check front ultrasonic only: false if clear
def check_collision():
    front_mm = ultrasonic_front.distance_mm()
    if front_mm < 0: # error
        print("ERROR: negative ultrasonic dist, continuing")
        return False
    elif front_mm < collisiondist: # collision
        print(f"Collision: {front_mm} mm room left")
        stop()
        print_oled()
        oled.text(f"COLLISION: {front_mm}", 0, 40)
        oled.show()
        return True
    return False

# Motor actions
def drive_forward():
    motor_left.set_forwards()
    motor_right.set_forwards()
    motor_left.duty(slow)
    motor_right.duty(slow)

def follow_line(): # Now can recover if only outside sensor sees line
    print_oled()
    oled.text("Follow line", 0, 40)
    oled.show()
    print("Follow line running")

    while not check_collision():
        mid = middle_IR.value()
        cl = center_left_IR.value()
        cr = center_right_IR.value()
        ol = outer_left_IR.value()
        or_ = outer_right_IR.value()

        # Exit and let process_sensors handle it
        if (ol == 1 and mid == 1) or (or_ == 1 and mid == 1):
            stop()
            break

        # Centred: go straight 
        if mid == 1 and cl == 0 and cr == 0:
            motor_left.set_forwards()
            motor_right.set_forwards()
            motor_left.duty(slow)
            motor_right.duty(slow)
            # OLED Notif
            print_oled()
            oled.text("Follow line: Straight", 0, 40)
            oled.show()

        # RIGHT deviation: pivot right
        elif (cr == 1 and cl == 0 and mid == 0) or (or_ == 1 and cl == 0 and mid == 0) or (cr == 1 and cl == 0 and mid == 1):
            motor_left.set_forwards()
            motor_right.set_backwards()
            motor_left.duty(outsidewheel)
            motor_right.duty(insidewheel)
            # OLED Notif
            print_oled()
            oled.text("Follow line: Right", 0, 40)
            oled.show()

        # LEFT deviation: pivot left
        elif (cl == 1 and cr == 0 and mid == 0) or (ol == 1 and cr == 0 and mid == 0) or (cl == 1 and cr == 0 and mid == 1): 
            motor_left.set_backwards()  # POTENTIAL NECESSARY FIX: REMOVE THE ONE WHERE MID = 1 AND CENTRE_SIDE = 1
            motor_right.set_forwards()
            motor_left.duty(insidewheel)
            motor_right.duty(outsidewheel)
            # OLED Notif
            print_oled()
            oled.text("Follow line: Left", 0, 40)
            oled.show()

        # RECOVERY: only outer sensor sees line → steer back
        elif (ol == 1 and cl == 0 and cr == 0 and mid == 0 and or_ == 0) or (or_ == 1 and cl == 0 and cr == 0 and mid == 0 and ol == 0):
            if ol == 1:
                motor_left.set_backwards()
                motor_right.set_forwards()
                motor_left.duty(insidewheel)
                motor_right.duty(outsidewheel)
                # OLED Notif
                print_oled()
                oled.text("Recover: Left", 0, 40)
                oled.show()

            elif or_ == 1:
                motor_left.set_forwards()
                motor_right.set_backwards()
                motor_left.duty(outsidewheel)
                motor_right.duty(insidewheel)
                # OLED Notif
                print_oled()
                oled.text("Recover: Right", 0, 40)
                oled.show()
        # Other situations: exit and let process_sensors handle it
        else:
            stop()
            break
        
        time.sleep(ontime)
        stop()
        time.sleep(offtime)

def handle_stub(side):
    print_oled()
    oled.text(f"handle stub: {side}", 0, 40)
    oled.show()
    print(f"Stub detected ({side})")
    if check_collision():
        return
    drive_forward()
    time.sleep(0.1) # actual time req UNKNOWN: NEED TO TEST
    stop()
    if middle_IR.value() == 1 and center_left_IR.value() == 0 and center_right_IR.value() == 0:
        print("Stub cleared")
        return
    elif middle_IR.value() == 0 and center_left_IR.value() == 0 and center_right_IR.value() == 0:
        print("Most likely 90 degree turn.")
        motor_left.set_backwards() 
        motor_right.set_forwards()
        motor_left.duty(slow)
        motor_right.duty(slow)
        time.sleep(0.2)
        stop()
    else:
        stop()
        print("ERROR: stub error, stopped.")
        return

def detect_y_intersection(side): # Essentially useless since never triggered as follow line will just cause it to take a random exit
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
    oled.text("No line", 0, 40)
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
        elif ratio < (1 - centretollerance): # Now actually moves in direction req to recover instead of just pivoting on spot and risking just tweaking out in a circle
            motor_right.set_backwards()
            motor_left.set_forwards()
            motor_left.duty(slow)
            motor_right.duty(slow)
            # OLED Notif
            print_oled()
            oled.text("No line: Adjust", 0, 40)
            oled.show()

            time.sleep(ontime)
            stop()
            time.sleep(offtime)
            # Move forwards in direction to recentre
            motor_left.set_forwards()
            motor_right.set_forwards()
            motor_left.duty(slow)
            motor_right.duty(slow)
            
        elif ratio > (1 + centretollerance):
            motor_right.set_forwards()
            motor_left.set_backwards()
            motor_left.duty(slow)
            motor_right.duty(slow)
            # OLED Notif
            print_oled()
            oled.text("No line: Adjust", 0, 40)
            oled.show()
            # Internal sleep to limit adjustment amount (need to test if this timeout is enough to prevent overshooting)
            time.sleep(ontime)
            stop()
            time.sleep(offtime)
            # Move forwards in direction to recentre
            motor_left.set_forwards()
            motor_right.set_forwards()
            motor_left.duty(slow)
            motor_right.duty(slow)

        time.sleep(ontime)
        stop()
        time.sleep(offtime)
        # If any sensor sees line, exit and let process_sensors handle it.
        if middle_IR.value() == 1 or outer_left_IR.value() == 1 or outer_right_IR.value() == 1 or center_left_IR.value() == 1 or center_right_IR.value() == 1:
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
        detect_y_intersection("LEFT") # Choose direction
    elif outer_left_IR.value() == 1 and outer_right_IR.value() == 0 and middle_IR.value() == 1:
        handle_stub("LEFT")
    elif outer_right_IR.value() == 1 and outer_left_IR.value() == 0 and middle_IR.value() == 1: # maybe add and centre right black too? (same for left one too?, probably noe necessary though...)
        handle_stub("RIGHT")
    elif outer_left_IR.value() == 0 and outer_right_IR.value() == 0 and center_left_IR.value() == 0 and center_right_IR.value() == 0 and middle_IR.value() == 0:
        no_line()
    elif (center_left_IR.value() == 0 and center_right_IR.value() == 0 and middle_IR.value() == 1) or (center_left_IR.value() == 0 and middle_IR.value() == 0 and center_right_IR.value() == 1) or (center_left_IR.value() == 1 and middle_IR.value() == 0 and center_right_IR.value() == 0):
        follow_line()
    else:
        print("ERROR: Unknown scenario.")
        # OLED Notif
        print_oled()
        oled.text("Unknown: Defaulting", 0, 40)
        oled.show()
        follow_line()
    time.sleep(ontime)

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
        
    if direction == 1: # right
        motor_right.set_backwards()
        for i in range(10):
            motor_left.duty(slow)
            motor_right.duty(slow)
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
            motor_left.duty(slow)
            motor_right.duty(slow)
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
        motor_left.duty(slow)
        motor_right.duty(slow)
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
        motor_left.duty(slow)
        motor_right.duty(slow) # Changed all roundabout motor duties to slow for consistency
        time.sleep(ontime)
        motor_right.duty(0)
        motor_left.duty(0)
        time.sleep(offtime)
        
    turn_on_path(direction)

def turn_on_path(direction):
    if direction == 1: # Right
        motor_right.set_backwards()
        while not middle_IR.value():
            motor_left.duty(slow)
            motor_right.duty(slow)
            time.sleep(ontime)
            motor_right.duty(0)
            motor_left.duty(0)
            time.sleep(offtime)
    elif direction == 0: # Left
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
    oled.text("Roundabout", 0, 40)
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
                    break  # Exit roundabout
                else:
                    # Direction == 2 (straight) now breaks after skipping to prevent infinite loop when select_direction returns 2
                    skip_distraction_line()
                    if path == 1:
                        turn_on_path(0)
                    else:
                        turn_on_path(1)
                    break
            else:
                # Error handling
                print_oled()
                oled.text("Roundabout: ERROR", 0, 40)
                oled.show()
                pass

def select_direction(): # DIRECTION SELECTOR FOR ROUNDABOUTS
    # 0 = left, 1 = right, 2 = straight
    richtungen = [1, 2, 0] # for every intersection in roundabout one direction (go left exit, then straight, then right if 1, 2, 0)
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
print("Initialising systems... (1s)")
# Display with OLED
print_oled()
oled.text("Initialising systems... (1s)", 0, 40)
oled.show()
time.sleep(1) # Wait 1s
print("Running ✅")
# Display with OLED again
print_oled()
oled.text("Running... ✅", 0, 40)
oled.show()
time.sleep(1)
drive_forward()

# Main loop
while True:        
    if not check_collision():
        process_sensors()
    else:
        stop()
        time.sleep(0.1)
