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

motor_left = Motor("Left", 11, 10, 7)
motor_right = Motor("Right", 9, 8, 6)

ultrasonic_front = sonic(3, 2)
ultrasonic_left  = sonic(5, 4)
ultrasonic_right = sonic(17, 16)

# globals
direction_counter = 0

# constants
slow = 40
outsidewheel = 45
insidewheel = 35
collisiondist = 50
centretollerance = 0.2
pivottimeout = 3.0

ontime = 0.02
offtime = 0.01

ontime_ms = int(ontime * 1000)
offtime_ms = int(offtime * 1000)

# OLED
i2c = SoftI2C(scl=Pin(5), sda=Pin(4))
oled = ssd1306.SSD1306_I2C(128, 64, i2c)

# Read Sensors

def read_sensors():
    return {
        "OL": outer_left_IR.value(),
        "CL": center_left_IR.value(),
        "M": middle_IR.value(),
        "CR": center_right_IR.value(),
        "OR": outer_right_IR.value()
    }

def any_line_detected():
    s = read_sensors()
    return s["OL"] or s["CL"] or s["M"] or s["CR"] or s["OR"]

# OLED 
def print_oled():
    s = read_sensors()
    oled.fill(0)
    oled.text(str(s["OL"]), 0, 10)
    oled.text(str(s["CL"]), 25, 10)
    oled.text(str(s["M"]), 50, 10)
    oled.text(str(s["CR"]), 75, 10)
    oled.text(str(s["OR"]), 100, 10)
    oled.show()

# CORE

def check_collision():
    print_oled()
    oled.text("collision", 0, 40)
    oled.show()

    front_mm = ultrasonic_front.distance_mm()
    if front_mm < 0:
        print("ERROR: ultrasonic")
        return False
    elif front_mm < collisiondist:
        stop()
        return True
    return False

def stop():
    print_oled()
    oled.text("stop", 0, 40)
    oled.show()
    motor_left.duty(0)
    motor_right.duty(0)

def drive_forward():
    motor_left.set_forwards()
    motor_right.set_forwards()
    motor_left.duty(slow)
    motor_right.duty(slow)

# LINE FOLLOW

def follow_line():
    print_oled()
    oled.text("follow", 0, 40)
    oled.show()

    while not check_collision():

        s = read_sensors()
        OL, CL, M, CR, OR = s["OL"], s["CL"], s["M"], s["CR"], s["OR"]

        if M == 1 and CL == 0 and CR == 0:
            motor_left.set_forwards()
            motor_right.set_forwards()
            motor_left.duty(slow)
            motor_right.duty(slow)

        elif (CR == 1 and CL == 0 and M in [0,1]):
            motor_left.set_forwards()
            motor_right.set_backwards()
            motor_left.duty(outsidewheel)
            motor_right.duty(insidewheel)

        elif (CL == 1 and CR == 0 and M in [0,1]):
            motor_left.set_backwards()
            motor_right.set_forwards()
            motor_left.duty(insidewheel)
            motor_right.duty(outsidewheel)

        else:
            stop()
            break

        # stub / intersection detection
        if OL == 1 or OR == 1:
            stop()
            break

        time.sleep_ms(ontime_ms)
        stop()
        time.sleep_ms(offtime_ms)

# STUB 
def handle_stub(side):
    print_oled()
    oled.text("stub", 0, 40)
    oled.show()

    if check_collision():
        return

    drive_forward()
    time.sleep_ms(100)
    stop()

    s = read_sensors()
    if s["M"] == 1 and s["CL"] == 0 and s["CR"] == 0:
        return
    elif s["M"] == 0 and s["CL"] == 0 and s["CR"] == 0:
        print("Likely 90 turn")
    else:
        stop()

# Y INTERSECTION

def detect_y_intersection(side):
    stop()
    time.sleep_ms(100)

    if side == "LEFT":
        motor_left.set_backwards()
        motor_right.set_forwards()
    else:
        motor_right.set_backwards()
        motor_left.set_forwards()

    motor_left.duty(slow)
    motor_right.duty(slow)

    time.sleep_ms(100)
    start = time.time()

    while True:
        if time.time() - start > pivottimeout:
            stop()
            return

        s = read_sensors()
        if s["M"] == 1:
            break

# No Line
def no_line():
    while True:
        if check_collision():
            return

        left_mm = ultrasonic_left.distance_mm()
        right_mm = ultrasonic_right.distance_mm()

        if left_mm <= 0 or right_mm <= 0:
            continue

        ratio = left_mm / right_mm

        if (1 - centretollerance) <= ratio <= (1 + centretollerance):
            motor_left.set_forwards()
            motor_right.set_forwards()
        elif ratio < (1 - centretollerance):
            motor_left.set_backwards()
            motor_right.set_forwards()
        else:
            motor_left.set_forwards()
            motor_right.set_backwards()

        motor_left.duty(slow)
        motor_right.duty(slow)

        time.sleep_ms(ontime_ms)
        stop()
        time.sleep_ms(offtime_ms)

        if any_line_detected():
            return

# Roundabout

def turn_on_path(direction):
    if direction == 1:
        motor_right.set_backwards()
    else:
        motor_left.set_backwards()

    while True:
        s = read_sensors()
        if s["M"]:
            break

        motor_left.duty(slow)
        motor_right.duty(slow)
        time.sleep_ms(20)
        stop()
        time.sleep_ms(20)

def turn_in_roundabout(direction):
    drive_forward()
    time.sleep_ms(150)

    if direction == 1:
        motor_right.set_backwards()
    else:
        motor_left.set_backwards()

    while not read_sensors()["M"]:
        motor_left.duty(slow)
        motor_right.duty(slow)
        time.sleep_ms(ontime_ms)
        stop()
        time.sleep_ms(offtime_ms)

def turn_out_roundabout(direction):
    drive_forward()
    time.sleep_ms(150)
    turn_on_path(direction)

def roundabout():
    stop()
    direction = select_direction()
    turn_in_roundabout(direction)
    path = direction

    while True:
        follow_line()
        s = read_sensors()

        if s["OR"] or s["OL"] or (s["CR"] and s["CL"] and s["M"]):
            if s["M"]:
                direction = select_direction()
                if direction in [0,1]:
                    turn_out_roundabout(direction)
                    break
                else:
                    skip_distraction_line()
                    turn_on_path(0 if path else 1)

# PROCESS SENSORS

def process_sensors():
    stop()
    s = read_sensors()

    if s["CL"] and s["CR"] and s["M"]:
        roundabout()

    elif (s["CL"] and s["CR"] and not s["M"]) or (s["OL"] and s["OR"] and not s["M"]):
        detect_y_intersection("LEFT")

    elif s["OL"] and not s["OR"] and s["M"]:
        handle_stub("LEFT")

    elif s["OR"] and not s["OL"] and s["M"]:
        handle_stub("RIGHT")

    elif not any_line_detected():
        no_line()

    else:
        follow_line()

    time.sleep_ms(20)

# UTILS

def select_direction():
    directions = [1,1,2,1]
    global direction_counter

    if direction_counter < len(directions):
        d = directions[direction_counter]
        direction_counter += 1
        return d
    else:
        direction_counter = 0
        return 1

def skip_distraction_line():
    drive_forward()
    time.sleep_ms(140)

# Main loop

motor_left.set_forwards()
motor_right.set_forwards()

print("Running")
drive_forward()

while True:
    if not check_collision():
        process_sensors()
    else:
        stop()
        time.sleep_ms(100)
