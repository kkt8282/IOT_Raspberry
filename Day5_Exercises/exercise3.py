import RPi.GPIO as GPIO
from time import sleep
import spidev

spi = spidev.SpiDev()
CDS_CHANNEL = 0

LED_1 = 4
LED_2 = 5
LED_3 = 14
LED_4 = 15
LED = [LED_1, LED_2, LED_4, LED_3]

def initMcp3208():
    spi.open(0, 0)
    spi.max_speed_hz = 1000000
    spi.mode = 3


def buildReadCommand(channel):

    startBit = 0x04
    singleEnded = 0x08

    configBit = [startBit | ((singleEnded | (channel & 0x07)) >> 2), (channel &0x07) << 6, 0x00]

    return configBit


def processAdcValue(result):
    byte2 = (result[1] & 0x0F)
    return (byte2 << 8) | result[2]

def analogRead(channel):
    if (channel > 7) or (channel < 0):
        return -1

    r = spi.xfer2(buildReadCommand(channel))
    adc_out = processAdcValue(r)
    return adc_out

def controlMcp3208(channel):
    analogVal = analogRead(channel)
    return analogVal

def readSensor(channel):
    return controlMcp3208(channel)

def main():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(LED, GPIO.OUT, initial=False)
    initMcp3208()
    print("Setup pin as outputs")

    try:
        while True:
            readVal = readSensor(CDS_CHANNEL)

            voltage = readVal * 4.096 / 4096
            print("CDS Val = %d\tVoltage=%f" % (readVal, voltage))
            if readVal <= 1332:
                LED_out = LED
                LEDSet = GPIO.HIGH
                GPIO.output(LED_out, LEDSet)
            elif readVal <= 1998:
                LEDSet = GPIO.HIGH
                GPIO.output(LED, LEDSet)
                LED_out = LED[3:4]
                LEDSet = GPIO.LOW
                GPIO.output(LED_out, LEDSet)
            elif readVal <= 2664:
                LEDSet = GPIO.HIGH
                GPIO.output(LED, LEDSet)
                LED_out = LED[2:4]
                LEDSet = GPIO.LOW
                GPIO.output(LED_out, LEDSet)
            elif readVal <= 3330:
                LEDSet = GPIO.HIGH
                GPIO.output(LED, LEDSet)
                LED_out = LED[1:4]
                LEDSet = GPIO.LOW
                GPIO.output(LED_out, LEDSet)
            elif readVal <= 3996:
                LED_out = LED
                LEDSet = GPIO.LOW
                GPIO.output(LED_out, LEDSet)
            
            sleep(0.5)

    
    except KeyboardInterrupt:
        GPIO.cleanup()
        spi.close()

if __name__ == '__main__':
    main()