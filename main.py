# Import necessary modules
import time
import _thread
import asyncio
from machine import Pin
from motor import Motor
from ultrasonic import sonic

# Setup sensors and motors
right_IR  = Pin(22, Pin.IN)
middle_IR = Pin(21, Pin.IN)
left_IR   = Pin(20, Pin.IN)
motor_left  = Motor("Left", 11, 10, 7) #probably need to find way to make them slower without duty reduction, maybe make a pwm of a pwm??
motor_right = Motor("Right", 9, 8, 6)
ultrasonic_front = sonic(3, 2)
ultrasonic_left  = sonic(5, 4) #same for these
ultrasonic_right = sonic(17, 16)
right_IR_active = False
left_IR_active = False
middle_IR_active = False

# Setup important constants
slow = 40
outsidewheel = 45
insidewheel = 35
collisiondist = 50 # mm 
centretollerance = 0.2 # tolerance for left/right ratio for no_line
pivottimeout = 3.0 # sec

def second_thread():
    asyncio.run(read_IR_sensors())

# Read IR sensors
async def read_IR_sensors():
    print("2 thread startet")
    await asyncio.gather(
        read_left_IR_sensor(),
        read_right_IR_sensor(),
        read_middle_IR_sensor()
    )
    
async def read_left_IR_sensor():
    global left_IR_active
    print ("left sensor evalu started")
    while True:
        left_val = left_IR.value()
        if (left_val == 1):
            left_IR_active = True
            await asyncio.sleep(0.3)
            left_IR_active = False
        await asyncio.sleep(0.05)

async def read_right_IR_sensor():
    global right_IR_active
    while True:
        right_val = right_IR.value()
        if (right_val == 1):
            right_IR_active = True
            await asyncio.sleep(0.3)
            right_IR_active = False
        await asyncio.sleep(0.05)

async def read_middle_IR_sensor():
    global middle_IR_active
    while True:
        middle_val = middle_IR.value()
        if (middle_val == 1):
            middle_IR_active = True
            await asyncio.sleep(0.3)
            middle_IR_active = False
        await asyncio.sleep(0.05)

# Check front ultrasonic only false if clear
def check_collision():
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
    print("follow line running")
    while not check_collision(): # true setzen + die wartezeit etwas hochsetzen unten
        # Centred: go straight 
        if middle_IR_active == 1 and left_IR_active == 0 and right_IR_active == 0:
            motor_left.set_forwards()
            motor_right.set_forwards()
            motor_left.duty(slow)
            motor_right.duty(slow)

        # RIGHT deviation: pivot right
        elif right_IR_active == 1 and left_IR_active == 0:
            motor_left.set_forwards()
            motor_right.set_backwards()
            motor_left.duty(outsidewheel)
            motor_right.duty(insidewheel)
            time.sleep(0.02)

        # LEFT deviation: pivot left
        elif left_IR_active == 1 and right_IR_active == 0: # in case middle is activated too, just make sure its not all 3 (roundabout)
            motor_left.set_backwards()
            motor_right.set_forwards()
            motor_left.duty(insidewheel)
            motor_right.duty(outsidewheel)
            time.sleep(0.02)

        # LINE LOST
        elif left_IR_active == 0 and right_IR_active == 0 and middle_IR_active == 0:
            stop()
            print("No line detected")
            break
        time.sleep(0.02)


def handle_stub(side):
    print(f"Stub detected ({side})")
    if check_collision(): # True = collision
        return
    drive_forward()
    time.sleep(0.1) #actual time req UNKNOWN NEED TO TEST!
    stop()
    if middle_IR_active == 1 and left_IR_active == 0 and right_IR_active == 0:
        print("Stub cleared")
        return
        # main loop takes over
    elif middle_IR_active == 0 and left_IR_active == 0 and right_IR_active == 0:
        print("Most likely 90 degree turn.")
    else:
        stop()
        print("ERROR: stub error, stopped.")
        return

def detect_y_intersection(side):
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
            if middle_IR_active == 1 and right_IR_active == 0:
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
            if middle_IR_active == 1 and left_IR_active == 0:
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
    stop()
    print("No line.")
    time.sleep(0.2)
    # Use ultrasonics
    
def stop():
    motor_left.duty(0)
    motor_right.duty(0)
    
# Detection
def process_sensors():
    stop()
    if left_IR_active == 0 and right_IR_active == 0 and middle_IR_active == 1: # Go straight
        follow_line()
    elif left_IR_active == 1 and right_IR_active == 1 and middle_IR_active == 1: #handle roundabout
        roundabout()
    elif left_IR_active == 1 and right_IR_active == 1 and middle_IR_active == 0: # This logic for detecting y intersections needs to be fixed (I don't think it works due to speed of motors being too high and either skipping or following random path instead of specified path due to this logic not even being triggered)
        detect_y_intersection("LEFT")  # change to "RIGHT" to take the other branch
    elif left_IR_active == 1 and right_IR_active == 0 and middle_IR_active == 1:
        handle_stub("LEFT")
    elif right_IR_active == 1 and left_IR_active == 0 and middle_IR_active == 1:
        handle_stub("RIGHT")
    elif left_IR_active == 0 and right_IR_active == 0 and middle_IR_active == 0:
        no_line()
    else:
        print("ERROR: Unknown senario.")
        follow_line()

def turn_vehicle(direction):
    motor_left.set_forwards()
    motor_right.set_forwards()
    motor_left.duty(50)
    motor_right.duty(50)
    time.sleep(0.25)
    motor_right.duty(0)
    motor_left.duty(0)
    
    if direction == 1:
        motor_left.duty(50)
        motor_right.set_backwards()
        motor_right.duty(50)
        time.sleep(0.33)
        motor_left.duty(0)
        motor_right.duty(0)
    else:
        motor_right.duty(50)
        motor_left.set_backwards()
        motor_left.duty(50)
        time.sleep(0.28)
        motor_right.duty(0)
        motor_left.duty(0)
        
def roundabout():
    print("Roundabout detected")
    stop()
    direction = select_direction()
    turn_vehicle(direction)
    
    while True:
        follow_line()
        if ((right_IR_active == 1) or (left_IR_active == 1)):
            if (middle_IR_active == 1): #helpful?????
                direction = select_direction()
                if ((direction == 1) or (direction == 0)): #1:right, 0:left
                    turn_vehicle(direction)
                    break
                else:
                    skip_distraction_line()
            else:
                #faulthandling
                pass

def select_direction():
    return 1
        
def skip_distraction_line():
    motor_left.set_forwards()
    motor_right.set_forwards()
    motor_left.duty(50)
    motor_right.duty(50)
    time.sleep(0.3)
    motor_right.duty(0)
    motor_left.duty(0)

#open second thread for sensor evaluation
_thread.start_new_thread(second_thread, ())
    
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
