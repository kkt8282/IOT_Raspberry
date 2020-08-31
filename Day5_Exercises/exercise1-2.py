import RPi.GPIO as GPIO
from time import sleep

LED_1 = 4
LED_2 = 5
LED_3 = 14
LED_4 = 15
LED = [LED_1, LED_2, LED_4, LED_3]

def main():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(LED, GPIO.OUT, initial=False)
    print("main() program running...")

    try: 
        while True:
            LEDNum = int(input("LED NUMBER: "))
            LEDSet = input("LED SET: ")
            if LEDSet == "ON":
                LEDSet = GPIO.HIGH
            if LEDSet == "OFF":
                LEDSet = GPIO.LOW
            GPIO.output(LED[LEDNum-1], LEDSet)
            
    except KeyboardInterrupt:
        GPIO.cleanup()

if __name__ == '__main__':
    main()