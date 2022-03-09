import RPi.GPIO as GPIO
import sys

input_pin = 16

def main():
   
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(input_pin, GPIO.IN)

    try:
        while True:
            value = GPIO.input(input_pin)
            sys.stdout.write("\033[F") 
            sys.stdout.write("\033[K")
            print(value)

    finally:
        GPIO.cleanup()

if __name__ == '__main__':
    main()
