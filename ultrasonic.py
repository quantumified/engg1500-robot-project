import machine
from machine import Pin
import time




class sonic:
    """
    A ``sonic`` object is used to control a HCSR04 ultrasonic sensor.
    """


    def __init__(self, trigger_pin, echo_pin, echo_timeout_us=4000*2/(340.29*1e-3)):
        """
        - trigger_pin: Output pin to send pulses
        - echo_pin: Input pin listens for reflection to estimate time of flight.
        - echo_timeout_us: Timeout in microseconds to listen to echo pin.
        By default the timeout is set to the max range limit of the sensor, 4m
        """
        # Initialise constants
        self.echo_timeout_us = int(echo_timeout_us)
        self.speed_sound = 370  # m/s


        # Initialise the TRIG output pin
        self.trigger = Pin(trigger_pin, mode=Pin.OUT)
        self.trigger.value(0)


        # Initialise the ECHO input pin
        self.echo = Pin(echo_pin, mode=Pin.IN)


    def distance_cm(self, sample_size=11, sample_wait=0.02):
        """Return an error corrected unrounded distance, in cm, of an object
        adjusted for temperature in Celcius.  The distance calculated
        is the median value of a sample of `sample_size` readings.


        Speed of readings is a result of two variables.  The sample_size
        per reading and the sample_wait (interval between individual samples).

        Example: To use a sample size of 5 instead of 11 will increase the
        speed of your reading but could increase variance in readings;

        value = sensor.Measurement(trig_pin, echo_pin)
        r = value.raw_distance(sample_size=5)

        Adjusting the interval between individual samples can also
        increase the speed of the reading.  Increasing the speed will also
        increase CPU usage.  Setting it too low will cause errors.  A default
        of sample_wait=0.1 is a good balance between speed and minimizing
        CPU usage.  It is also a safe setting that should not cause errors.

        e.g.

        r = value.raw_distance(sample_wait=0.03)
        """

        sample = []

        for wrong_distance_reading in range(sample_size):
            self.trigger.value(0)
            time.sleep(sample_wait)
            self.trigger.value(1)
            time.sleep(0.00001)
            self.trigger.value(0)
            echo_status_counter = 1
            while self.echo.value() == 0:
                if echo_status_counter < 1000:
                    sonar_signal_off = time.time()
                    echo_status_counter += 1
                else:
                    print("Echo pulse was not received")
                    break
            while self.echo.value() == 1:
                sonar_signal_on = time.time()
            time_passed = sonar_signal_on - sonar_signal_off
            distance_cm = time_passed * ((self.speed_sound * 100) / 2)
            sample.append(distance_cm)
        sorted_sample = sorted(sample)
        # Only cleanup the pins used to prevent clobbering
        # any others in use by the program
        return sorted_sample[sample_size // 2]

    def distance_mm(self, sample_size=6, sample_wait=0.02):
        """
        Estimate distance to obstacle in front of the ultrasonic sensor in mm.
        Sends a 10us pulse to 'trigger' pin and listens on 'echo' pin.
        We use the method `machine.time_pulse_us()` to count the microseconds
        passed before the echo is received.
        The time of flight is used to calculate the estimated distance.
        """
        
        sample = []

        for distance_reading in range(sample_size):
            # Send a 10us HIGH pulse to trigger the ultrasonic burst
            self.trigger.value(0)
            time.sleep(sample_wait)
            self.trigger.value(1)
            time.sleep_us(10)
            self.trigger.value(0)
            # Read length of time pulse
            duration = machine.time_pulse_us(self.echo, 1, self.echo_timeout_us)
            # Calculate the distance in mm, based on the delay before we hear
            # an echo and the constant speed of sound.
            # Note: duration is halved as the audio wave must travel there and back.
            mm = 0.5 * duration * 1e-6 * self.speed_sound * 1e3
            
            if duration < 0:
                mm = 300
            sample.append(mm)
        sorted_sample = sorted(sample)
        # Only cleanup the pins used to prevent clobbering
        # any others in use by the program
        return sorted_sample[sample_size // 2]