import RPi.GPIO as gpio
import time

class HCSR04:
    trig = 0
    echo = 0
    const_cm = 17014.50
    const_in = 6698.62
    const_ft = 558.2

    def __init__(self, trig, echo):
        self.trig = trig
        self.echo = echo

        gpio.setmode(gpio.BOARD)
        gpio.setup(self.trig, gpio.OUT)
        gpio.setup(self.echo, gpio.IN)
        gpio.output(self.trig, False)

        # Sleep for 0.3 s for the sensor to settle
        time.sleep(0.3)

    def delete(self):
        gpio.cleanup()
        print("INFO: GPIO cleanup complete")
        time.sleep(0.1)

    def measure(self, samples, unit):
        count = 0
        acc = 0
        valid_samples = 0  # Track how many valid readings we get

        while count < samples:
            gpio.output(self.trig, True)
            time.sleep(0.00001)
            gpio.output(self.trig, False)

            timeout = time.time() + 0.02
            while gpio.input(self.echo) == 0:
                pulse_start = time.time()
                if pulse_start > timeout:
                    print("WARNING: No echo detected")
                    return -1  # Avoid infinite loop

            timeout = time.time() + 0.02
            while gpio.input(self.echo) == 1:
                pulse_end = time.time()
                if pulse_end > timeout:
                    print("WARNING: Echo signal stuck")
                    return -1

            pulse_duration = pulse_end - pulse_start

            if unit == "cm":
                distance = pulse_duration * self.const_cm
            elif unit == "in":
                distance = pulse_duration * self.const_in
            elif unit == "ft":
                distance = pulse_duration * self.const_ft

            if distance < 2 or distance > 400:
                print("WARNING: Ignoring invalid reading:", distance)
                continue  # Ignore this measurement

            acc += distance
            valid_samples += 1
            count += 1

        if valid_samples == 0:
            print("ERROR: No valid sonar readings detected")
            return -1  # Return error if all samples were bad

        acc = round(acc / valid_samples, 2)
        return acc
