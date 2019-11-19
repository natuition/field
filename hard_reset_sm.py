#!/usr/bin/env python

import RPi.GPIO as GPIO
import time

# Pin Definitions
output_pin = 18  # BOARD pin 12, BCM pin 18


def main():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(output_pin, GPIO.OUT, initial=GPIO.LOW)

    try:
        while True:
            action = input("Type h to enable power, l to disable, anything else to exit: ")
            if action == "h":
                print("Setting HIGH value")
                GPIO.output(output_pin, GPIO.HIGH)
            elif action == "l":
                print("Setting LOW value")
                GPIO.output(output_pin, GPIO.LOW)
            else:
                print("Exit command received")
                break
    finally:
        GPIO.cleanup()


if __name__ == '__main__':
    main()
