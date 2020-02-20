BCM_OUTPUT_PIN = 19
FREQUENCY_HZ = 100


class LEDGPIOAdapter:
    def __init__(self, bcm_output_pin, frequency_hz):
        from RPi import GPIO
        self._output_pin = bcm_output_pin
        self._frequency = frequency_hz
        GPIO.setwarnings(False)  # do not show any warnings
        GPIO.setmode(GPIO.BCM)  # we are programming the GPIO by BCM pin numbers.
        GPIO.setup(BCM_OUTPUT_PIN, GPIO.OUT)  # initialize requested GPIO pin as an output.
        self._pwm = GPIO.PWM(BCM_OUTPUT_PIN, FREQUENCY_HZ)  # requested GPIO as PWM output, with requested frequency
        self._pwm.start(0)  # generate PWM signal with 0% duty cycle

    def set_light_level(self, percent_value):
        if percent_value < 0 or percent_value > 100:
            raise ValueError("")
        self._pwm.ChangeDutyCycle(percent_value)


def test_LEDGPIOAdapter():
    import time
    led = LEDGPIOAdapter(BCM_OUTPUT_PIN, FREQUENCY_HZ)

    while True:
        for power_value in list(range(1, 101)) + list(range(99, -1, -1)):
            led.set_light_level(power_value)
            time.sleep(0.1)


if __name__ == '__main__':
    test_LEDGPIOAdapter()
