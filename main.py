# ------------------------------------------------------------------
# no_line with timeout and corrected adjustment directions
# ------------------------------------------------------------------
def no_line():
    print_oled("No line")
    print("No line: ultrasonic sensing")

    start_time = time.time()
    NO_LINE_TIMEOUT = 5.0

    # --- Helper to detect line and adjust until centered ---
    def detect_and_center():
        """Check all IR sensors. If line is seen, pivot to bring it to middle sensor.
        Returns True if line was found and centered (or already centered),
        False if no line is seen at all."""
        mid = read_ir_sensor(middle_IR)
        cl  = read_ir_sensor(center_left_IR)
        cr  = read_ir_sensor(center_right_IR)
        ol  = read_ir_sensor(outer_left_IR)
        or_ = read_ir_sensor(outer_right_IR)

        # No line at all
        if mid == 0 and cl == 0 and cr == 0 and ol == 0 and or_ == 0:
            return False

        # Already centered
        if mid == 1:
            drive_forward()
            time.sleep(0.1)
            stop()
            return True

        # Line on left side (CL or OL) -> pivot left to center it
        if cl == 1 or ol == 1:
            motor_left.set_backwards()
            motor_right.set_forwards()
            pivot_start = time.time()
            while time.time() - pivot_start < 1.0:   # 1 second timeout
                motor_left.duty(slow)
                motor_right.duty(slow)
                time.sleep(0.05)
                stop()
                time.sleep(0.05)
                if read_ir_sensor(middle_IR) == 1:
                    drive_forward()
                    time.sleep(0.1)
                    stop()
                    return True
            # Even if middle wasn't reached, we saw the line – return True to stop searching
            return True

        # Line on right side (CR or OR) -> pivot right to center it
        if cr == 1 or or_ == 1:
            motor_left.set_forwards()
            motor_right.set_backwards()
            pivot_start = time.time()
            while time.time() - pivot_start < 1.0:
                motor_left.duty(slow)
                motor_right.duty(slow)
                time.sleep(0.05)
                stop()
                time.sleep(0.05)
                if read_ir_sensor(middle_IR) == 1:
                    drive_forward()
                    time.sleep(0.1)
                    stop()
                    return True
            return True

        return False
    # ------------------------------------------------------------

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
        if detect_and_center():
            return

        motor_left.set_forwards()
        motor_right.set_backwards()
        for i in range(find_track_var):
            motor_left.duty(slow)
            motor_right.duty(slow)
            time.sleep(0.02)
            stop()
            time.sleep(0.1)
            if detect_and_center():
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
            if detect_and_center():
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
                if detect_and_center():
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
                if detect_and_center():
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
