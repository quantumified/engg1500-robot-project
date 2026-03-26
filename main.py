# Import necessary modules
import time
from machine import Pin
from motor import Motor
from ultrasonic import sonic

# Setup sensors and motors
right_IR  = Pin(22, Pin.IN)
middle_IR = Pin(21, Pin.IN)
left_IR   = Pin(20, Pin.IN)
motor_left  = Motor("Left", 11, 10, 7)
motor_right = Motor("Right", 9, 8, 6)
ultrasonic_front = sonic(3, 2)
ultrasonic_left  = sonic(5, 4)
ultrasonic_right = sonic(17, 16) 

# Setup important constants
slow = 40
outsidewheel = 45
insidewheel = 35
movetime = 0.30 #sec
collisiondist = 50 # mm 
centretollerance = 0.2 # tolerance for left/right ratio for no_line
pivottimeout = 3.0 # sec

# Read IR sensors
def read_IRsensors():
    global left_val, right_val, middle_val
    left_val = left_IR.value()
    right_val = right_IR.value()
    middle_val = middle_IR.value()

# Check front ultrasonic only — returns True if collision imminent, False if clear
def check_collision():
    front_mm = ultrasonic_front.distance_mm()
    if front_mm < collisiondist:
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

def stop():
    motor_left.duty(0)
    motor_right.duty(0)

# Line follow
def follow_line():
    motor_left.set_forwards()
    motor_right.set_forwards()
    motor_left.duty(40)
    motor_right.duty(40)

    while not check_collision():
        right_val  = right_IR.value()
        middle_val = middle_IR.value()
        left_val   = left_IR.value()

        motor_left.set_forwards()
        motor_right.set_forwards()
        motor_left.duty(40)
        motor_right.duty(40)

        if right_val == 1:
            while right_val == 1:
                motor_left.duty(45)
                motor_right.duty(35)
                motor_right.set_backwards()
                right_val = right_IR.value()
            motor_left.duty(40)
            motor_right.duty(40)
            motor_right.set_forwards()

        elif left_val == 1:
            while left_val == 1:
                motor_right.duty(45)
                motor_left.duty(35)
                motor_left.set_backwards()
                left_val = left_IR.value()
            motor_left.duty(40)
            motor_right.duty(40)
            motor_left.set_forwards()

        elif middle_val == 0 and left_val == 0 and right_val == 0:
            # All sensors white wait then confirm before stop
            time.sleep(0.4)
            right_val  = right_IR.value()
            middle_val = middle_IR.value()
            left_val   = left_IR.value()
            if middle_val == 0 and left_val == 0 and right_val == 0:
                stop()
                break

def handle_stub(side):
    print(f"Stub detected ({side})")
    if check_collision(): # True = collision
        return
    drive_forward()
    time.sleep(movetime)
    read_IRsensors()
    if middle_val == 1 and left_val == 0 and right_val == 0:
        print("Stub cleared — resuming")
        # main loop takes over
    else:
        stop()
        print("Stub: error")

def detect_roundabout():
    stop()
    print("Roundabout (LEFT=BLACK, CENTRE=BLACK, RIGHT=BLACK)")
    # roundabout navigation; to be implemented

def detect_y_intersection(side):
    stop()
    print(f"Y Intersection — taking {side}")

    # Brief pause
    time.sleep(0.1)

    if side == "LEFT":
        # Pivot left on the spot
        motor_left.set_backwards()
        motor_right.set_forwards()
        motor_left.duty(slow)
        motor_right.duty(slow)

        time.sleep(0.2)

        start = time.time()
        while True:
            if time.time() - start > pivottimeout:
                print("ERROR: Y-intersection LEFT pivot timed out")
                stop()
                return
            middle_val = middle_IR.value()
            right_val  = right_IR.value()
            if middle_val == 1 and right_val == 0:
                break
            time.sleep(0.01)

    elif side == "RIGHT":
        # Pivot right on the spot
        motor_right.set_backwards()
        motor_left.set_forwards()
        motor_right.duty(slow)
        motor_left.duty(slow)

        time.sleep(0.2)

        start = time.time()
        while True:
            if time.time() - start > pivottimeout:
                print("ERROR: Y-intersection RIGHT pivot timed out")
                stop()
                return
            middle_val = middle_IR.value()
            left_val   = left_IR.value()
            if middle_val == 1 and left_val == 0:
                break
            time.sleep(0.01)

    else:
        print(f"ERROR detect_y_intersection: {side}, stopping.")
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
    
def process_sensors():
    read_IRsensors()

    if left_val == 1 and right_val == 1 and middle_val == 1:
        detect_roundabout()
    elif left_val == 1 and right_val == 1 and middle_val == 0:
        detect_y_intersection("LEFT")  # change to "RIGHT" to take the other branch
    elif left_val == 1 and right_val == 0 and middle_val == 1:
        handle_stub("LEFT")
    elif right_val == 1 and left_val == 0 and middle_val == 1:
        handle_stub("RIGHT")
    elif left_val == 0 and right_val == 0 and middle_val == 0:
        no_line()
    else:
        follow_line()


# Start-up
motor_left.set_forwards()
motor_right.set_forwards()
print("Starting in 2 seconds...")
time.sleep(2)
print("Running")
drive_forward()

# Main loop
while True:
    if not check_collision(): # if false runs this
        process_sensors()
        time.sleep(0.2)
    else:
        stop()
        time.sleep(0.2)
