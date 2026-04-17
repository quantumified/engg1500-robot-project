# Import necessary modules
import random
import time
from machine import Pin, SoftI2C
from motor import Motor
from ultrasonic import sonic
import ssd1306

# ─────────────────────────────────────────────
# Hardware setup
# ─────────────────────────────────────────────
middle_IR       = Pin(21, Pin.IN)
center_right_IR = Pin(22, Pin.IN)
center_left_IR  = Pin(20, Pin.IN)
outer_right_IR  = Pin(18, Pin.IN)
outer_left_IR   = Pin(19, Pin.IN)
motor_left  = Motor("Left",  11, 10, 7)
motor_right = Motor("Right",  9,  8, 6)
ultrasonic_front = sonic(14, 15)
ultrasonic_left  = sonic(3,  2)
ultrasonic_right = sonic(12, 13)

# ─────────────────────────────────────────────
# Constants
# ─────────────────────────────────────────────
slow             = 41
outsidewheel     = 45
insidewheel      = 35
collisiondist    = 50     # mm
centretollerance = 0.4    # ratio band for no_line straight driving
pivottimeout     = 3.0    # sec – max time for any sensor-guided pivot
ontime           = 0.02   # motor-on pulse duration
offtime          = 0.01   # motor-off pause duration

# Number of consecutive stopped reads required before believing it
# is a real roundabout (robot is stationary during this check).
ROUNDABOUT_CONFIRM_TICKS = 8

# ─────────────────────────────────────────────
# Roundabout direction table
# One entry per roundabout encounter (not per exit passed inside).
# 0 = take left exit, 1 = take right exit, 2 = skip (go to next exit)
# ─────────────────────────────────────────────
ROUNDABOUT_PLAN = [1, 2, 0]   # 1st encounter→right, 2nd→skip/straight, 3rd→left
direction_counter = 0          # indexes into ROUNDABOUT_PLAN

# ─────────────────────────────────────────────
# OLED
# ─────────────────────────────────────────────
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

# ─────────────────────────────────────────────
# Low-level helpers
# ─────────────────────────────────────────────
def stop():
    motor_left.duty(0)
    motor_right.duty(0)

def read_all():
    """Return (ol, cl, mid, cr, or_) sensor tuple."""
    return (outer_left_IR.value(), center_left_IR.value(),
            middle_IR.value(), center_right_IR.value(), outer_right_IR.value())

def any_line():
    return any(s == 1 for s in read_all())

def check_collision():
    """Return True (and stop) if obstacle is within collisiondist."""
    dist = ultrasonic_front.distance_mm()
    if dist < 0:
        print("WARN: negative ultrasonic reading, ignoring")
        return False
    if dist < collisiondist:
        stop()
        print_oled(f"COLLISION:{dist}mm")
        return True
    return False

def pulse_forward(n=1):
    """Drive straight for n on/off pulses."""
    motor_left.set_forwards()
    motor_right.set_forwards()
    for _ in range(n):
        motor_left.duty(slow)
        motor_right.duty(slow)
        time.sleep(ontime)
        stop()
        time.sleep(offtime)

def pulse_pivot_right(n=1):
    """Pivot right (left fwd, right back) for n pulses."""
    motor_left.set_forwards()
    motor_right.set_backwards()
    for _ in range(n):
        motor_left.duty(outsidewheel)
        motor_right.duty(insidewheel)
        time.sleep(ontime)
        stop()
        time.sleep(offtime)

def pulse_pivot_left(n=1):
    """Pivot left (right fwd, left back) for n pulses."""
    motor_left.set_backwards()
    motor_right.set_forwards()
    for _ in range(n):
        motor_left.duty(insidewheel)
        motor_right.duty(outsidewheel)
        time.sleep(ontime)
        stop()
        time.sleep(offtime)

def seek_line_pivot(direction, timeout=pivottimeout):
    """
    Pivot in `direction` (1=right, 0=left) until middle_IR sees the
    line or timeout expires. Returns True if line found.
    """
    start = time.time()
    if direction == 1:
        motor_left.set_forwards()
        motor_right.set_backwards()
    else:
        motor_left.set_backwards()
        motor_right.set_forwards()

    while True:
        if time.time() - start > timeout:
            stop()
            print("ERROR: seek_line_pivot timed out")
            return False
        if middle_IR.value() == 1:
            stop()
            return True
        motor_left.duty(outsidewheel if direction == 1 else insidewheel)
        motor_right.duty(insidewheel if direction == 1 else outsidewheel)
        time.sleep(ontime)
        stop()
        time.sleep(offtime)

# ─────────────────────────────────────────────
# follow_line
# ─────────────────────────────────────────────
def follow_line():
    """
    Follow the line until an intersection / junction is detected or
    a collision occurs. Every branch falls through to the pulse cycle.
    """
    print_oled("Follow line")
    print("Follow line")

    while not check_collision():
        ol, cl, mid, cr, or_ = read_all()

        # ── Exit: wide-line or junction detected ───────────────────────
        if (cl == 1 and cr == 1) or (ol == 1 and or_ == 1) or \
           (ol == 1 and mid == 1) or (or_ == 1 and mid == 1):
            stop()
            return

        # ── No line at all ─────────────────────────────────────────────
        if ol == 0 and cl == 0 and mid == 0 and cr == 0 and or_ == 0:
            stop()
            return

        # ── Straight ───────────────────────────────────────────────────
        if mid == 1 and cl == 0 and cr == 0 and ol == 0 and or_ == 0:
            motor_left.set_forwards()
            motor_right.set_forwards()
            motor_left.duty(slow)
            motor_right.duty(slow)
            print_oled("FL: Straight")

        # ── Right deviation ────────────────────────────────────────────
        elif (cr == 1 and cl == 0) or (or_ == 1 and cl == 0 and mid == 0):
            motor_left.set_forwards()
            motor_right.set_backwards()
            motor_left.duty(outsidewheel)
            motor_right.duty(insidewheel)
            print_oled("FL: Right")

        # ── Left deviation ─────────────────────────────────────────────
        elif (cl == 1 and cr == 0) or (ol == 1 and cr == 0 and mid == 0):
            motor_left.set_backwards()
            motor_right.set_forwards()
            motor_left.duty(insidewheel)
            motor_right.duty(outsidewheel)
            print_oled("FL: Left")

        # ── Anything else: exit to process_sensors ─────────────────────
        else:
            stop()
            return

        # ── Speed-limiting pulse (ALL branches reach here) ─────────────
        time.sleep(ontime)
        stop()
        time.sleep(offtime)

# ─────────────────────────────────────────────
# Roundabout detection & handling
# ─────────────────────────────────────────────
def is_roundabout():
    """
    Robot must already be STOPPED before calling this.
    Reads sensors ROUNDABOUT_CONFIRM_TICKS times with small pauses
    (no movement at all) and returns True only if all five sensors
    stay lit throughout. This prevents the robot drifting into the
    blank centre of the roundabout during confirmation.
    """
    stop()  # Guarantee stopped — belt-and-braces
    for tick in range(ROUNDABOUT_CONFIRM_TICKS):
        ol, cl, mid, cr, or_ = read_all()
        if not (ol == 1 and cl == 1 and mid == 1 and cr == 1 and or_ == 1):
            print(f"Roundabout confirm failed at tick {tick}")
            return False
        time.sleep(0.015)   # short stationary pause between reads, no movement
    print("Roundabout confirmed")
    return True

def select_exit():
    """
    Returns the planned exit choice for this roundabout encounter:
      0 = take the left exit
      1 = take the right exit
      2 = skip this exit (continue around to the next one)
    Advances direction_counter so the NEXT roundabout gets the next entry.
    Wraps around if we've exhausted the plan.
    """
    global direction_counter
    if direction_counter < len(ROUNDABOUT_PLAN):
        choice = ROUNDABOUT_PLAN[direction_counter]
    else:
        direction_counter = 0
        choice = ROUNDABOUT_PLAN[0]
    direction_counter += 1
    print(f"Roundabout plan: encounter {direction_counter}, choice={choice}")
    return choice

def advance_past_blob(n=8):
    """Creep forward to clear the thick intersection line before pivoting."""
    pulse_forward(n)

def roundabout():
    print_oled("Roundabout!")
    print("Roundabout: entering")
    stop()

    # Decide the exit strategy for THIS entire roundabout encounter.
    # select_exit() is called ONCE here and never again inside the loop,
    # so direction_counter advances by exactly 1 per roundabout.
    planned_exit = select_exit()
    print(f"Planned exit direction: {planned_exit}")

    # ── Enter the roundabout ───────────────────────────────────────────
    # Creep forward past the entry blob then pivot onto the inner track.
    # We always enter by turning right (following the roundabout clockwise).
    advance_past_blob(7)
    seek_line_pivot(1)   # pivot right onto roundabout inner track

    exits_passed = 0  # how many exits we have encountered and skipped

    # ── Circulate: follow inner track, handle each exit ───────────────
    while True:
        follow_line()

        # After follow_line exits, assess what we're sitting on
        ol, cl, mid, cr, or_ = read_all()

        # Detect an exit (outer sensors lit, or full-blob again)
        exit_detected = (or_ == 1 or ol == 1 or
                         (cl == 1 and cr == 1 and mid == 1))

        if not exit_detected:
            # Lost line or minor noise — try to recover inside roundabout
            if not any_line():
                print_oled("RBT: no line")
                no_line()
            else:
                pulse_forward(2)
            continue

        # ── An exit is in front of us ──────────────────────────────────
        if planned_exit == 2:
            # "Straight" / skip: we want the SECOND exit we encounter.
            # Skip the first one and take the next.
            if exits_passed == 0:
                exits_passed += 1
                print_oled("RBT: skip exit 1")
                # Drive past this exit without turning off
                advance_past_blob(7)
                seek_line_pivot(1)   # re-acquire inner track
                continue
            else:
                # This is the second exit — take it (turn right off)
                print_oled("RBT: take exit 2")
                advance_past_blob(8)
                seek_line_pivot(1)
                return

        elif planned_exit == 1:
            # Take the FIRST exit to the right
            print_oled("RBT: take right")
            advance_past_blob(8)
            seek_line_pivot(1)
            return

        elif planned_exit == 0:
            # Take the FIRST exit to the left
            print_oled("RBT: take left")
            advance_past_blob(8)
            seek_line_pivot(0)
            return

        else:
            # Fallback — shouldn't happen
            print("ERROR: unknown planned_exit value, taking right")
            advance_past_blob(8)
            seek_line_pivot(1)
            return

# ─────────────────────────────────────────────
# handle_stub
# ─────────────────────────────────────────────
def handle_stub(side):
    print_oled(f"Stub: {side}")
    print(f"Stub detected ({side})")
    if check_collision():
        return

    # Creep forward until the stub is no longer under the outer sensor
    # or until a 90-degree turn becomes apparent.
    start = time.time()
    while time.time() - start < 1.0:
        ol, cl, mid, cr, or_ = read_all()
        if mid == 1 and cl == 0 and cr == 0:
            print("Stub cleared")
            return
        if (cl == 1 or cr == 1) and mid == 0:
            print("Stub → likely 90° turn")
            seek_line_pivot(0 if side == "LEFT" else 1)
            return
        pulse_forward(1)

    print("WARN: stub timeout, continuing")

# ─────────────────────────────────────────────
# detect_y_intersection
# ─────────────────────────────────────────────
def detect_y_intersection(side):
    print_oled(f"Y-inter: {side}")
    print(f"Y-intersection: {side}")
    stop()
    pulse_forward(5)          # clear the junction blob
    seek_line_pivot(0 if side == "LEFT" else 1)
    pulse_forward(3)

# ─────────────────────────────────────────────
# no_line
# ─────────────────────────────────────────────
def no_line():
    print_oled("No line")
    print("No line: ultrasonic wall-following")

    start_time = time.time()
    TIMEOUT = 5

    while True:
        if time.time() - start_time > TIMEOUT:
            print("no_line: timeout")
            return

        if check_collision():
            return

        if any_line():
            print("Line reacquired")
            return

        left_mm  = ultrasonic_left.distance_mm()
        right_mm = ultrasonic_right.distance_mm()

        if left_mm is None or right_mm is None or left_mm <= 0 or right_mm <= 0:
            pulse_forward(1)
            continue

        ratio = left_mm / right_mm

        # Very lost: spin randomly then advance
        if ratio > 4 or ratio < 0.25:
            print_oled("NL: VERY LOST")
            if random.choice([True, False]):
                pulse_pivot_right(3)
            else:
                pulse_pivot_left(3)
            pulse_forward(2)
            continue

        # Closer to left wall (left_mm small → ratio < 1) → steer RIGHT
        if ratio < (1 - centretollerance):
            motor_left.set_forwards()
            motor_right.set_forwards()
            motor_left.duty(slow)
            motor_right.duty(int(slow * 0.6))
            time.sleep(ontime)
            stop()
            time.sleep(offtime)

        # Closer to right wall → steer LEFT
        elif ratio > (1 + centretollerance):
            motor_left.set_forwards()
            motor_right.set_forwards()
            motor_left.duty(int(slow * 0.6))
            motor_right.duty(slow)
            time.sleep(ontime)
            stop()
            time.sleep(offtime)

        # Centred → straight
        else:
            pulse_forward(1)

# ─────────────────────────────────────────────
# process_sensors  (dispatcher)
# ─────────────────────────────────────────────
def process_sensors():
    stop()
    ol, cl, mid, cr, or_ = read_all()

    # ── Possible roundabout: ALL centre sensors lit ────────────────────
    # Stop first, then confirm stationary. If confirmation fails it was
    # a glitch/T-junction so fall through to Y-intersection handling.
    if cl == 1 and cr == 1 and mid == 1:
        stop()                      # <-- stopped BEFORE is_roundabout()
        if is_roundabout():
            roundabout()
            return
        # Confirmation failed: treat as Y-intersection
        detect_y_intersection("LEFT")
        return

    # ── Y / T intersection ────────────────────────────────────────────
    if (cl == 1 and cr == 1 and mid == 0) or \
       (ol == 1 and or_ == 1 and mid == 0):
        detect_y_intersection("LEFT")
        return

    # ── Left stub ─────────────────────────────────────────────────────
    if ol == 1 and or_ == 0 and mid == 1:
        handle_stub("LEFT")
        return

    # ── Right stub ────────────────────────────────────────────────────
    if or_ == 1 and ol == 0 and mid == 1:
        handle_stub("RIGHT")
        return

    # ── No line ───────────────────────────────────────────────────────
    if ol == 0 and cl == 0 and mid == 0 and cr == 0 and or_ == 0:
        no_line()
        return

    # ── Normal line ───────────────────────────────────────────────────
    if mid == 1 or cl == 1 or cr == 1:
        follow_line()
        return

    # ── Unknown ───────────────────────────────────────────────────────
    print("WARN: Unknown sensor state, defaulting to follow_line")
    print_oled("Unknown: FL")
    follow_line()

# ─────────────────────────────────────────────
# Start-up
# ─────────────────────────────────────────────
motor_left.set_forwards()
motor_right.set_forwards()
print("Initialising systems... (1s)")
print_oled("Initialising...")
time.sleep(1)
print("Running")
print_oled("Running...")
time.sleep(1)
pulse_forward(5)   # gentle start

# ─────────────────────────────────────────────
# Main loop
# ─────────────────────────────────────────────
while True:
    if not check_collision():
        process_sensors()
    else:
        stop()
        time.sleep(0.1)
