import board
import digitalio
import RPi.GPIO as GPIO
import adafruit_character_lcd.character_lcd as character_lcd
import GPIO_EX
from time import sleep

lcd_rs = digitalio.DigitalInOut(board.D22)
lcd_en = digitalio.DigitalInOut(board.D24)
lcd_d7 = digitalio.DigitalInOut(board.D21)
lcd_d6 = digitalio.DigitalInOut(board.D26)
lcd_d5 = digitalio.DigitalInOut(board.D20)
lcd_d4 = digitalio.DigitalInOut(board.D19)

lcd_columns = 16
lcd_rows = 2

lcd = character_lcd.Character_LCD_Mono(lcd_rs, lcd_en, lcd_d4, lcd_d5, lcd_d6, lcd_d7, lcd_columns, lcd_rows)

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

colTable = [COL0_PIN, COL1_PIN, COL2_PIN]
rowTable = [ROW0_PIN, ROW1_PIN, ROW2_PIN, ROW3_PIN]

def initTextlcd():
    lcd.clear()
    lcd.home()
    lcd.cursor_position(0,0)
    sleep(1.0)

def displayText(text='', col=0, row=0):
    lcd.cursor_position(col,row)
    lcd.message = text

def clearTextlcd():
    lcd.clear()
    lcd.message = 'clear LCD\nGoodbye!'
    sleep(2.0)
    lcd.clear()

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
    return keyData

def main():
    password_1 = []
    password_2 = []

    initTextlcd()
    print("start textlcd program...")

    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)

    initKeypad()
    print("setup keypad pin")
    try:
        while True:   
            keyData = readKeypad()
            if keyData == -1:
                continue
            if len(password_1) != 4:
                password_1.append(str(keyData))
                print("\r\nKeypad Data : {}".format(keyData))
                line = ''.join(password_1)
                lcd.clear()
                displayText(line, 0,0)
            else:
                line = ''.join(password_1)
                displayText(line, 0,0)
                password_2.append(str(keyData))
                if len(password_2) == 4:
                    line2 = ''.join(password_2)
                    if line == line2:
                        ans = 'CORRECT'
                    else:
                        ans = 'FAIL'
                    displayText(ans, 0,1)
                    password_1 = []
                    password_2 = []
                    sleep(5)
                    lcd.clear()

    except KeyboardInterrupt:
        GPIO.cleanup()

if __name__ == '__main__':
    main()