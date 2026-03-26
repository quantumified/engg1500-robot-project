# Import necessary modules
import time
from machine import Pin
from motor import Motor
from ultrasonic import sonic

# Setup sensors and motors
print("Initialising")
right_IR  = Pin(22, Pin.IN)
middle_IR = Pin(21, Pin.IN)
left_IR   = Pin(20, Pin.IN)
motor_left = Motor("Left", 11, 10, 7)
motor_right = Motor("Right", 9, 8, 6)
ultrasonic = sonic(3, 2)

# Setup important constants
slow = 40
outsidewheel = 45
insidewheel = 35
movetime = 0.30 #sec
wallthreshold = 500 # mm
collisiondist = 50

# All functions
# Read sensors
def read_IRsensors():
    global  left_val, right_val, middle_val
    left_val = left_IR.value()
    right_val = right_IR.value()
    middle_val = middle_IR.value()

def read_ultrasonics():
    return (ultrasonic.distance_mm())

# Check front ultrasonic sensor
def check_collision():
    ultrasonic_mm = ultrasonic.distance_mm()
    if ultrasonic_mm < collisiondist:
        print(f"Collision: {ultrasonic_mm} mm room left")
        stop()
        return False
    else:
        return True

# Motor actions
def drive_forward():
    motor_left.set_forwards()
    motor_right.set_forwards()
    motor_left.duty(slow)
    motor_right.duty(slow)

def stop():
    motor_left.duty(0)
    motor_right.duty(0)

def turn_right():
    motor_left.set_forwards()
    motor_right.set_backwards()
    motor_left.duty(outsidewheel)
    motor_right.duty(insidewheel)

def turnleft():
    motor_right.set_forwards()
    motor_left.set_backwards()
    motor_right.duty(outsidewheel)
    motor_left.duty(insidewheel)

# line follow (from charlie)
def follow_line():
    motor_left.duty(30)
    motor_right.duty(30)

    while not check_collision:
    
        right_val = right_IR.value()
        middle_val = middle_IR.value()
        left_val = left_IR.value()
    
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
            time.sleep(0.4)
            right_val = right_IR.value()
            middle_val = middle_IR.value()
            left_val = left_IR.value()
        
            if middle_val == 0 and left_val == 0 and right_val == 0:
                motor_left.duty(0)
                motor_right.duty(0)
                break
        

# rest
def handle_stub(side):
    print("stub")
    if not check_collision():
        return    
    drive_forward()
    time.sleep(movetime)

    read_IRsensors()

    if middle_val == 1 and left_val == 0 and right_val == 0:
        print("Stub cleared")
    else: #elif all is white, then probably 90 degree turn
        stop()
        print("error")

def detect_roundabout():
    stop()
    print("Roundabout (LEFT=BLACK, CENTRE=BLACK, RIGHT=BLACK)")
    # roundabout navigation

def detect_y_intersection():
    stop()
    print("Y Intersection (LEFT=BLACK, CENTRE=WHITE, RIGHT=BLACK)")
    # Y-intersection navigation


def no_line(): #
    ultrasonic_mm = read_ultrasonics()
#    print(f"ultrasonic side required:  {left_val}, {middle_val}, {right_val}")
    stop()

def process_sensors():
    read_IRsensors()
#    print(f"process sensors: {left_val}, {middle_val}, {right_val}")

    if left_val == 1 and right_val == 1 and middle_val == 0:
        detect_y_intersection()
    elif left_val == 0 and right_val == 0 and middle_val == 0:
        no_line()
    elif left_val == 1 and right_val == 1 and middle_val == 1:
        detect_roundabout()
    elif left_val == 1 and right_val == 0 and middle_val == 1:
        handle_stub("LEFT")
    elif right_val == 1 and left_val == 0 and middle_val == 1:
        handle_stub("RIGHT")
    else:
        follow_line()
            # line following
        
            
# Start-up
print("Starting in 2 seconds...")
motor_left.set_forwards()
motor_right.set_forwards()
time.sleep(2)
print("Running")
drive_forward()

#  Main loop (need to fix ts bum ass logic)
while True:
    if check_collision() == True: #false (no collision)
        process_sensors()
        time.sleep(0.2)
    else: #obstacle
        stop()
        time.sleep(0.2)


