import RPi.GPIO as GPIO
from time import sleep

LED_1 = 4
LED_2 = 5
LED_3 = 14
LED_4 = 15

def main():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(LED_1, GPIO.OUT, initial=False)
    GPIO.setup(LED_2, GPIO.OUT, initial=False)
    GPIO.setup(LED_3, GPIO.OUT, initial=False)
    GPIO.setup(LED_4, GPIO.OUT, initial=False)
    print("main() program running...")

    try: 
        while True:
            GPIO.output(LED_1, GPIO.HIGH)
            sleep(0.5)
            GPIO.output(LED_2, GPIO.HIGH)
            sleep(0.5)
            GPIO.output(LED_3, GPIO.HIGH)
            sleep(0.5)
            GPIO.output(LED_4, GPIO.HIGH)
            sleep(0.5)
            GPIO.output(LED_1, GPIO.LOW)
            sleep(0.5)
            GPIO.output(LED_2, GPIO.LOW)
            sleep(0.5)
            GPIO.output(LED_3, GPIO.LOW)
            sleep(0.5)
            GPIO.output(LED_4, GPIO.LOW)
            sleep(0.5)

    except KeyboardInterrupt:
        GPIO.cleanup()

if __name__ == '__main__':
    main()