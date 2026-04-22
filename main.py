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
ultrasonic_front = sonic(14, 15)
ultrasonic_left  = sonic(3, 2) #same for these
ultrasonic_right = sonic(12, 13)

#global counters
direction_counter = 0
main_loop_iteration = 0
EARLY_LOST_THRESHOLD = 20          # Number of main loop cycles to check
forced_right_done = False          # Only do this once
returning_home = False

# Setup important constants
slow = 47
slow_right = 53
outsidewheel = 50
outsidewheel_left = 50
insidewheel = 40
outsidewheel_slow = 45
insidewheel_slow = 40
collisiondist = 50 # mm 
centretollerance = 0.2 # tolerance for left/right ratio for no_line
pivottimeout = 3.0 # sec
ontime = 0.02
offtime = 0.2

find_track_var = 18
init_drive_out_of_garage = 0.4
time_burst_out_of_roundabout = 0.2

# OLED bullshit
#You can choose any other combination of I2C pins
i2c = SoftI2C(scl=Pin(5), sda=Pin(4))
oled = ssd1306.SSD1306_I2C(128, 64, i2c)
oled.invert(True)

def print_oled(label=""):
    oled.fill(0)
    oled.text("OL:" + str(outer_left_IR.value()),  0,  10)
    oled.text("CL:" + str(center_left_IR.value()), 45, 10)
    oled.text("M:"  + str(middle_IR.value()),       85, 10)
    oled.text("CR:" + str(center_right_IR.value()),  0, 20)
    oled.text("OR:" + str(outer_right_IR.value()),  45, 20)
    if label:
        oled.text(label[:20], 0, 40)
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
    front_mm = ultrasonic_front.distance_mm()
    if front_mm < 0: #error
        print("ERROR: negative ultrasonic dist, continuing")
        return False
    elif front_mm < collisiondist: #collision
        print_oled(f"COLLISION:{dist}mm")
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
    print_oled("Follow line")

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
        if (mid == 1 and cl == 0 and cr == 0) or (mid == 1 and or_ == 1 and cr == 1) or (mid == 1 and ol == 1 and cl == 1):
            motor_left.set_forwards()
            motor_right.set_forwards()
            motor_left.duty(slow)
            motor_right.duty(slow_right)
            print_oled("FL: Straight")

        # Right deviation
        elif (cr == 1 and cl == 0 and mid == 0) or (or_ == 1 and cl == 0 and mid == 0):
            motor_left.set_forwards()
            motor_right.set_forwards()
            motor_left.duty(outsidewheel_left)
            motor_right.duty(insidewheel)
            print_oled("FL: Right")
        elif (cr == 1 and cl == 0 and mid == 1):
            motor_left.set_forwards()
            motor_right.set_forwards()
            motor_left.duty(outsidewheel)
            motor_right.duty(insidewheel_slow)
            print_oled("FL: Right")

        # Left deviation
        elif (cl == 1 and cr == 0 and mid == 0) or (ol == 1 and cr == 0 and mid == 0):
            motor_left.set_forwards()
            motor_right.set_forwards()
            motor_left.duty(insidewheel)
            motor_right.duty(outsidewheel)
            print_oled("FL: Left")
        elif (cl == 1 and cr == 0 and mid == 1):
            motor_left.set_forwards()
            motor_right.set_forwards()
            motor_left.duty(insidewheel_slow)
            motor_right.duty(outsidewheel_slow)
            print_oled("FL: Left")

        # Recovery: only outer sensor sees line
        elif (ol == 1 and cr == 0 and mid == 0 and or_ == 0) or (or_ == 1 and cl == 0 and mid == 0 and ol == 0):
            if ol == 1:
                motor_left.set_backwards()
                motor_right.set_forwards()
                motor_left.duty(insidewheel)
                motor_right.duty(outsidewheel)
                print_oled("Recover: Left")
            elif or_ == 1:
                motor_left.set_forwards()
                motor_right.set_backwards()
                motor_left.duty(outsidewheel)
                motor_right.duty(insidewheel)
                print_oled("Recover: Right")
        else:
            # Unexpected pattern; exit and re-evaluate
            stop()
            break

        time.sleep(3 * ontime)
        stop()
        time.sleep(offtime / 2)

# ------------------------------------------------------------------
# handle_stub with correct direction
# ------------------------------------------------------------------
def handle_stub(side):
    print_oled(f"handle stub: {side}")
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

# Function that's called once the robot has passed all roundabouts, and hits the y-intersection from the left.
# Only adjusts to the right, so it avoids accidentally just going back onto the course.

def exit_course():
    """
    Follow line while returning home. Only corrects to the right.
    If line is lost, calls no_line() to navigate into garage using ultrasonics.
    """
    print_oled("Exit Course")
    
    while not check_collision():
        mid = read_ir_sensor(middle_IR)
        cl  = read_ir_sensor(center_left_IR)
        cr  = read_ir_sensor(center_right_IR)
        ol  = read_ir_sensor(outer_left_IR)
        or_ = read_ir_sensor(outer_right_IR)

        # If all sensors are off, line is lost – use hallway/no_line code
        if mid == 0 and cl == 0 and cr == 0 and ol == 0 and or_ == 0:
            no_line()
            break

        # If the line is on the left side (cl or ol active) – pivot RIGHT to bring it back
        if cl == 1 or ol == 1:
            motor_left.set_forwards()
            motor_right.set_backwards()
            motor_left.duty(insidewheel)   # adjust these values if needed
            motor_right.duty(insidewheel)
            print_oled("Exit: Right")
        else:
            # Otherwise go straight (even if line is on right – we don't turn left)
            motor_left.set_forwards()
            motor_right.set_forwards()
            motor_left.duty(slow)
            motor_right.duty(slow)
            print_oled("Exit: Straight")

        time.sleep(ontime)
        stop()
        time.sleep(offtime)

# ------------------------------------------------------------------
# no_line with timeout and corrected adjustment directions
# ------------------------------------------------------------------
def no_line():
    print_oled("No line")
    print("No line: ultrasonic sensing")

    start_time = time.time()
    NO_LINE_TIMEOUT = 5.0
    
    if (ultrasonic_left.distance_mm() > 150) and (ultrasonic_right.distance_mm() > 150):
        print("just no line")
        print_oled("NL: Searching")
        dont_turn_to_other_side = False
        motor_left.set_forwards()
        motor_right.set_forwards()
        for i in range(8):
            motor_left.duty(slow)
            motor_right.duty(slow)
            time.sleep(0.02)
            stop()
        if read_ir_sensor(middle_IR) or read_ir_sensor(center_left_IR) or read_ir_sensor(center_right_IR):
            return
                
        motor_left.set_forwards()
        motor_right.set_backwards()
        for i in range(find_track_var):
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
        for i in range(find_track_var):
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
            motor_left.set_backwards()
            motor_right.set_forwards()
            for i in range(find_track_var * 2):
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
            for i in range(find_track_var * 2):
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
            motor_left.set_forwards()
            motor_right.set_forwards()
            for i in range(10):
                motor_left.duty(slow)
                motor_right.duty(slow)
                time.sleep(0.02)
                stop()
                time.sleep(0.1)
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
            print_oled("US Err. Skip cycle")
            time.sleep(0.05)
            continue

        if right_mm == 0:
            right_mm = 1
        ratio = left_mm / right_mm

        # Center tolerance
        if (left_mm > 150) and (right_mm > 150):
            print("no line after hallway")
            print_oled("NL after hallway")
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
            print_oled("No Line: Turn Right")
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
            print_oled("No Line: Turn Left")
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
    
def stop():
    motor_left.duty(0)
    motor_right.duty(0)
    
# ------------------------------------------------------------------
# process_sensors with debounced reads
# ------------------------------------------------------------------
def process_sensors():
    stop()
    print_oled("Process Sensors")
    mid = read_ir_sensor(middle_IR)
    cl  = read_ir_sensor(center_left_IR)
    cr  = read_ir_sensor(center_right_IR)
    ol  = read_ir_sensor(outer_left_IR)
    or_ = read_ir_sensor(outer_right_IR)

    #print(f"process_sensors: mid={mid} cl={cl} cr={cr} ol={ol} or={or_}")
    
    if returning_home:
        if (ol + cl + mid + cr + or_) > 2 or (cl == 1 and cr == 1):
            exit_course()
            return

    if (cl == 1 and cr == 1 and mid == 1) or (cl == 1 and cr == 1 and mid == 0) or (ol == 1 and or_ == 1 and mid == 0):
        roundabout() #y-intesection is also handled with the roundabout method
    elif ol == 1 and or_ == 0 and mid == 1:
        follow_line()
        #handle_stub("LEFT")
    elif or_ == 1 and ol == 0 and mid == 1:
        follow_line()
        #handle_stub("RIGHT")
    elif cl == 0 and cr == 0 and mid == 0:
        no_line()
    elif (cl == 0 and cr == 0 and mid == 1) or (cl == 0 and mid == 0 and cr == 1) or (cl == 1 and mid == 0 and cr == 0):
        follow_line()
    else:
        print("ERROR: Unknown scenario, defaulting to follow_line")
        print_oled("Unknown. Def follow line")
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
    
# _________________________________________
# Roundabout Bullshit. So much redundancy
# _________________________________________


def recover_from_lost_in_roundabout():
    print_oled("Recover Roundabout")
    # Pivot left a bit
    motor_left.set_backwards()
    motor_right.set_forwards()
    for _ in range(5):
        motor_left.duty(slow)
        motor_right.duty(slow)
        time.sleep(0.05)
        stop()
        time.sleep(0.05)
        if read_ir_sensor(middle_IR) or read_ir_sensor(center_left_IR) or read_ir_sensor(center_right_IR):
            return
    # Pivot right a bit
    motor_left.set_forwards()
    motor_right.set_backwards()
    for _ in range(10):
        motor_left.duty(slow)
        motor_right.duty(slow)
        time.sleep(0.05)
        stop()
        time.sleep(0.05)
        if read_ir_sensor(middle_IR) or read_ir_sensor(center_left_IR) or read_ir_sensor(center_right_IR):
            return
    # Drive forward a little
    drive_forward()
    time.sleep(0.3)
    stop()

def follow_line_roundabout():
    # Follow line inside roundabout. Returns True if an exit is seen
    while not check_collision():
        mid = read_ir_sensor(middle_IR)
        cl  = read_ir_sensor(center_left_IR)
        cr  = read_ir_sensor(center_right_IR)
        ol  = read_ir_sensor(outer_left_IR)
        or_ = read_ir_sensor(outer_right_IR)

        # If we see a possible exit pattern, return to let roundabout() handle it
        if (ol == 1 and mid == 1) or (or_ == 1 and mid == 1) or (cl == 1 and cr == 1 and mid == 1):
            return True

        # All sensors off – lost line (maybe we drifted inside roundabout)
        if mid == 0 and cl == 0 and cr == 0 and ol == 0 and or_ == 0:
            return False   # signal that we lost the line

        # Normal line following (similar to follow_line but without breaking on stubs)
        if mid == 1 and cl == 0 and cr == 0:
            print_oled("Round Straight")
            motor_left.set_forwards()
            motor_right.set_forwards()
            motor_left.duty(slow)
            motor_right.duty(slow)
        elif (cr == 1 and cl == 0) or (or_ == 1 and cl == 0):
            print_oled("Round Right")
            motor_left.set_forwards()
            motor_right.set_backwards()
            motor_left.duty(outsidewheel_left)
            motor_right.duty(insidewheel)
            time.sleep(0.08)
            motor_right.set_forwards()
        elif (cl == 1 and cr == 0) or (ol == 1 and cr == 0):
            print_oled("Round Left")
            motor_left.set_forwards()
            motor_right.set_forwards()
            motor_left.duty(insidewheel)
            motor_right.duty(outsidewheel)
        else:
            # Unexpected pattern, just keep last command briefly
            pass

        time.sleep(3 * ontime)
        stop()
        time.sleep(offtime / 2)
    return False

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
    global returning_home
    print_oled(f"Exit: {'RIGHT' if direction == 1 else 'LEFT'}")
    
    # 1. Drive forward a bit to center over the intersection
    motor_left.set_forwards()
    motor_right.set_forwards()
    motor_left.duty(slow)
    motor_right.duty(slow)
    time.sleep(0.15)   # adjust as needed
    stop()
    time.sleep(0.05)
    
    # 2. Pivot in the chosen direction until the middle sensor sees the line again
    if direction == 1:   # right
        motor_left.set_forwards()
        motor_right.set_backwards()
    elif direction == 0: # left
        motor_left.set_backwards()
        motor_right.set_forwards()
    else:
        return
    
    start = time.time()
    timeout = 5.0  # seconds – prevents infinite loop
    
    while time.time() - start < timeout:
        motor_left.duty(slow)
        motor_right.duty(slow)
        time.sleep(0.08)
        stop()
        time.sleep(offtime)
        if read_ir_sensor(middle_IR) == 1:
            break
    
    # 3. Drive forward a little to fully clear the exit
    motor_left.set_forwards()
    motor_right.set_forwards()
    motor_left.duty(slow)
    motor_right.duty(slow)
    time.sleep(time_burst_out_of_roundabout)
    stop()
    if direction_counter >= 7:
        returning_home = True
    print_oled("Exit complete")

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
    print_oled("Roundabout")
    stop()
    direction = select_direction()   # first call: initial entry turn
    turn_in_roundabout(direction)
    path = direction

    while True:
        # Use the dedicated roundabout follower
        exit_seen = follow_line_roundabout()
        
        if not exit_seen:
            # Lost the line – try to recover by small search
            # (maybe just pivot slightly until line found)
            recover_from_lost_in_roundabout()
            continue

        # Now check if we are at a real exit
        mid = read_ir_sensor(middle_IR)
        ol  = read_ir_sensor(outer_left_IR)
        or_ = read_ir_sensor(outer_right_IR)
        cl  = read_ir_sensor(center_left_IR)
        cr  = read_ir_sensor(center_right_IR)

        # Valid exit condition (middle + at least one outer, or all three centers)
        if (mid == 1 and (ol == 1 or or_ == 1)) or (cl == 1 and cr == 1 and mid == 1):
            direction = select_direction()   # get exit direction
            if direction in (0, 1):
                turn_out_roundabout(direction)
                time.sleep(0.2)
                break
            else:
                # direction == 2 means skip this exit (straight)
                skip_distraction_line()
                # keep path variable unchanged
        else:
            # False trigger – continue following
            pass
            
def follow_line_roundi():
    print_oled("Follow Line Round")
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
    richtungen = [0, 0, 0, 2, 0, 0, 0]#for every intersection in roundabout one direction
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
print_oled("Initialising systems... (1s)")
time.sleep(1)
print("Running ✅")
print_oled("Running... ✅")
time.sleep(1)
drive_forward()

# Main loop
while True:
    main_loop_iteration += 1

    if not check_collision():
        # --- EARLY LOST FALLBACK ---
        if (not forced_right_done and 
            main_loop_iteration <= EARLY_LOST_THRESHOLD and
            middle_IR.value() == 0 and 
            center_left_IR.value() == 0 and 
            center_right_IR.value() == 0 and
            outer_left_IR.value() == 0 and 
            outer_right_IR.value() == 0):
            
            print("Early lost detected – forcing RIGHT turn")
            print_oled("Y-int. Go Right")
            
            # Drive forward a bit to clear the intersection
            drive_forward()
            time.sleep(init_drive_out_of_garage)   # Adjust distance as needed
            stop()
            
            # Perform the right turn (pivot on the spot)
            turn_vehicle(1)   # 1 = right turn
            
            forced_right_done = True
            # After this, normal process_sensors() will take over
            # (It will likely find the line again and follow it.)
            continue
        # --- END FALLBACK ---

        process_sensors()
    else:
        stop()
        time.sleep(0.1)

