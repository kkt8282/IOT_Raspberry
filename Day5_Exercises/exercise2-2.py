import RPi.GPIO as GPIO
from time import sleep
import GPIO_EX

ROW0_PIN = 0
ROW1_PIN = 1
ROW2_PIN = 2
ROW3_PIN = 3
COL0_PIN = 4
COL1_PIN = 5
COL2_PIN = 6

COL_NUM = 3
ROW_NUM = 4

g_preData = 0

LED_1 = 4
LED_2 = 5
LED_3 = 14
LED_4 = 15

LED = [LED_1, LED_2, LED_4, LED_3]
colTable = [COL0_PIN, COL1_PIN, COL2_PIN]
rowTable = [ROW0_PIN, ROW1_PIN, ROW2_PIN, ROW3_PIN]
LED_status = [False, False, False, False]

def initKeypad():
    for i in range(0, COL_NUM):
        GPIO_EX.setup(colTable[i], GPIO_EX.IN)
    for i in range(0, ROW_NUM):
        GPIO_EX.setup(rowTable[i], GPIO_EX.OUT)


def selectRow(rowNum):
    for i in range(0, ROW_NUM):
        if rowNum == (i + 1):
            GPIO_EX.output(rowTable[i], GPIO_EX.HIGH)
            sleep(0.001)
        else:
            GPIO_EX.output(rowTable[i], GPIO_EX.LOW)
            sleep(0.001)
    return rowNum

def readCol():
    keypadstate = -1
    for i in range(0, COL_NUM):
        inputKey = GPIO_EX.input(colTable[i])
        if inputKey:
            keypadstate = keypadstate + (i + 2)
            sleep(0.5)
    return keypadstate

def readKeypad():
    global g_preData
    keyData = -1

    runningStep = selectRow(1)
    row1Data = readCol()
    selectRow(0)
    sleep(0.001)
    if (row1Data != -1):
        keyData = row1Data

    if runningStep == 1:
        if keyData == -1:
            runningStep = selectRow(2)
            row2Data = readCol()
            selectRow(0)
            sleep(0.001)
            if (row2Data != -1):
                keyData = row2Data + 3

    if runningStep == 2:
        if keyData == -1 :
            runningStep = selectRow(3)
            row3Data = readCol()
            selectRow(0)
            sleep(0.001)
            if (row3Data != -1):
                keyData = row3Data + 6

    if runningStep == 3:
        if keyData == -1 :
            runningStep = selectRow(4)
            row4Data = readCol() #row4Data는 col값
            selectRow(0)
            sleep(0.001)
            if (row4Data != -1):
                Row4List = ['*', 0, '#']
                keyData = Row4List[row4Data - 1]



    sleep(0.1)

    if keyData == -1:
        return -1

    if g_preData == keyData:
        g_preData = -1
        return -1
    g_preData = keyData

    print("\r\nKeypad Data : {}".format(keyData))

    return keyData

def main():
    global LED_status
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(LED, GPIO.OUT, initial=False)

    initKeypad()
    print("setup keypad pin")
    try:
        while True:
            keyData = readKeypad()
            LEDNum = keyData
            if LEDNum == -1:
                continue
            if LEDNum <= 4:
                if LED_status[LEDNum - 1] == False:
                    LEDSet = GPIO.HIGH
                    LED_status[LEDNum - 1] = True
                else: 
                    LEDSet = GPIO.LOW
                    LED_status[LEDNum - 1] = False
                GPIO.output(LED[LEDNum-1], LEDSet)
            else:
                LEDSet = GPIO.LOW
                GPIO.output(LED, LEDSet)
                LED_status = [False, False, False, False]

            
    except KeyboardInterrupt:
        GPIO.cleanup()

if __name__ == '__main__':
    main()