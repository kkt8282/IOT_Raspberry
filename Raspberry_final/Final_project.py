import numpy as np
import cv2
import pickle
import board
import digitalio
import adafruit_character_lcd.character_lcd as character_lcd
import RPi.GPIO as GPIO
import GPIO_EX
import threading
import spidev
import adafruit_dht
from flask import Flask
from time import sleep

password_1 = '0000'
spi = spidev.SpiDev()
CDS_CHANNEL = 0

FAN_PIN1 = 18
FAN_PIN2 = 27

dhtDevice = adafruit_dht.DHT11(board.D17)

LED_1 = 4
LED_2 = 5
LED_3 = 14
LED_4 = 15
LED = [LED_1, LED_2, LED_4, LED_3]

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

app = Flask(__name__)

detect_state = False
detect_state2 = False
detect_state3 = False
autoCDS_status = False
autoFan_status = False
auto_status = False
mic_status = 0

PIR_PIN = 7
pirState = 0
ON = 1
OFF = 0

scale = [230, 261, 294, 329, 349, 392, 440, 493, 523, 553]

melodyList = [8,8,9,8,8,8,8,9,8,6,4,0,8,8,9,8,8,8,8,9,8,6,4,0]
noteDurations = [0.4, 0.2, 0.2, 0.4, 0.4, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2,
                0.4, 0.2, 0.2, 0.4, 0.4, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2]

openDoorBeep = [2,0,4,7]
noteDurations1 = [0.2,0.2,0.2,0.2]

failpassword = [8,5,8,5,8,5]
failpasswordDurations = [0.2,0.2,0.2,0.2,0.2,0.2]

notDetectfor3sec = [7,4,7,4]
noteDurations2 = [0.3,0.6,0.3,0.6]

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup([4,5,14,15,18,27,PIR_PIN],GPIO.OUT,initial=GPIO.LOW)
GPIO_EX.setup(PIR_PIN, GPIO_EX.IN)
pwm = GPIO.PWM(PIR_PIN, 100)


def AutoMode():
    while True:
        try:
            if auto_status == False:
                return
            temperature_c = dhtDevice.temperature
            humidity = dhtDevice.humidity
            readVal = readSensor(CDS_CHANNEL)
            voltage = readVal * 4.096 / 4096
            
            print("Temp : {:.1f}C   Humidity: {}%".format(temperature_c, humidity))
            print("CDS Val = %d\tVoltage=%f" % (readVal, voltage))
            
            if temperature_c >= 28 or humidity >= 60:
                onFan()
            else:
                offFan()

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
            sleep(1.0)
        except RuntimeError as error:
            print(error.args[0])
def AutoMode2():
    while True:
        try:
            if auto_status == False:
                return
            temperature_c = dhtDevice.temperature
            humidity = dhtDevice.humidity
            readVal = readSensor(CDS_CHANNEL)
            voltage = readVal * 4.096 / 4096
            
            print("Temp : {:.1f}C   Humidity: {}%".format(temperature_c, humidity))
            print("CDS Val = %d\tVoltage=%f" % (readVal, voltage))
            
            if temperature_c >= 25 or humidity >= 50:
                onFan()
            else:
                offFan()

            if readVal <= 1000:
                LED_out = LED
                LEDSet = GPIO.HIGH
                GPIO.output(LED_out, LEDSet)
            elif readVal <= 2000:
                LEDSet = GPIO.HIGH
                GPIO.output(LED, LEDSet)
                LED_out = LED[3:4]
                LEDSet = GPIO.LOW
                GPIO.output(LED_out, LEDSet)
            elif readVal <= 3000:
                LEDSet = GPIO.HIGH
                GPIO.output(LED, LEDSet)
                LED_out = LED[2:4]
                LEDSet = GPIO.LOW
                GPIO.output(LED_out, LEDSet)
            elif readVal <= 4000:
                LEDSet = GPIO.HIGH
                GPIO.output(LED, LEDSet)
                LED_out = LED[1:4]
                LEDSet = GPIO.LOW
                GPIO.output(LED_out, LEDSet)
            elif readVal <= 5000:
                LED_out = LED
                LEDSet = GPIO.LOW
                GPIO.output(LED_out, LEDSet)
            sleep(1.0)
        except RuntimeError as error:
            print(error.args[0])

def AutoFan():
    while True:
        try:
            if autoFan_status == False:
                return
            temperature_c = dhtDevice.temperature
            humidity = dhtDevice.humidity
            
            print("Temp : {:.1f}C   Humidity: {}%".format(temperature_c, humidity))
            sleep(1.0)
            if temperature_c >= 28 or humidity >= 60:
                onFan()
            else:
                offFan()
        except RuntimeError as error:
            print(error.args[0])
        
def AutoCDS():
    while True:
        if autoCDS_status == False:
            return
        readVal = readSensor(CDS_CHANNEL)
        voltage = readVal * 4.096 / 4096
        print("CDS Val = %d\tVoltage=%f" % (readVal, voltage))
        sleep(1.0)
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

def AutoFan2():
    while True:
        try:
            if autoFan_status == False:
                return
            temperature_c = dhtDevice.temperature
            humidity = dhtDevice.humidity
            
            print("Temp : {:.1f}C   Humidity: {}%".format(temperature_c, humidity))
            sleep(1.0)
            if temperature_c >= 25 or humidity >= 50:
                onFan()
            else:
                offFan()
        except RuntimeError as error:
            print(error.args[0])
        
def AutoCDS2():
    while True:
        if autoCDS_status == False:
            return
        readVal = readSensor(CDS_CHANNEL)
        voltage = readVal * 4.096 / 4096
        print("CDS Val = %d\tVoltage=%f" % (readVal, voltage))
        sleep(1.0)
        if readVal <= 1000:
            LED_out = LED
            LEDSet = GPIO.HIGH
            GPIO.output(LED_out, LEDSet)
        elif readVal <= 2000:
            LEDSet = GPIO.HIGH
            GPIO.output(LED, LEDSet)
            LED_out = LED[3:4]
            LEDSet = GPIO.LOW
            GPIO.output(LED_out, LEDSet)
        elif readVal <= 3000:
            LEDSet = GPIO.HIGH
            GPIO.output(LED, LEDSet)
            LED_out = LED[2:4]
            LEDSet = GPIO.LOW
            GPIO.output(LED_out, LEDSet)
        elif readVal <= 4000:
            LEDSet = GPIO.HIGH
            GPIO.output(LED, LEDSet)
            LED_out = LED[1:4]
            LEDSet = GPIO.LOW
            GPIO.output(LED_out, LEDSet)
        elif readVal <= 5000:
            LED_out = LED
            LEDSet = GPIO.LOW
            GPIO.output(LED_out, LEDSet)

def onFan():
    GPIO.output(FAN_PIN1, GPIO.HIGH)
    GPIO.output(FAN_PIN2, GPIO.LOW)

def offFan():
    GPIO.output(FAN_PIN1, GPIO.LOW)
    GPIO.output(FAN_PIN2, GPIO.LOW)

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

def controlKeypad(detect_state3):
    global password_1
    while detect_state3:
            try:
                if (mic_status == 1):
                    password_2 = []
                    while True:   
                        keyData = readKeypad()
                        if keyData == -1:
                            continue
                        password_2.append(str(keyData))
                        print("\r\nKeypad Data : {}".format(keyData))
                        line2 = ''.join(password_2)
                        lcd.clear()
                        displayText(line2, 0,0)
                        if len(password_2) == 4:
                            line2 = ''.join(password_2)
                            if password_1 == line2:
                                ans = 'CORRECT'
                                playBuzzer(openDoorBeep, noteDurations1)
                            else:
                                ans = 'FAIL'
                                playBuzzer(failpassword, failpasswordDurations)
                            displayText(ans, 0,1)
                            password_2 = []
                            sleep(5)
                            lcd.clear()
                            break
                else:
                    line = 'ACCESS DENIED'
                    lcd.clear()
                    displayText(line, 0,0)
                    sleep(3)
                    lcd.clear()
            except KeyboardInterrupt:
                GPIO.cleanup()

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


def playBuzzer(melodyList, noteDurations):
    pwm.start(100)
    pwm.ChangeDutyCycle(50)

    for i in range(len(melodyList)):
        pwm.ChangeFrequency(scale[melodyList[i]])
        sleep(noteDurations[i])
    pwm.stop()

def readPir(detect_state2):
    global pirState
    while detect_state2:
        input_state = GPIO_EX.input(PIR_PIN)
        if input_state == True:
            if pirState == 0:
                #print("\r\nMotion Detected")
                pass
            pirState = 1
            sleep(3)
            if mic_status == 0:
                playBuzzer(notDetectfor3sec, noteDurations2)
        else:
            if pirState == 1:
                #print("\r\nMotion Ended")
                pass
            pirState = 0
        sleep(0.5)
    

def controlDevice(detect_state):
    global mic_status
    face_cascade = cv2.CascadeClassifier('cascades/data/haarcascade_frontalface_alt2.xml')
    eye_cascade = cv2.CascadeClassifier('cascades/data/haarcascade_eye.xml')
    smile_cascade = cv2.CascadeClassifier('cascades/data/haarcascade_smile.xml')

    recognizer = cv2.face.LBPHFaceRecognizer_create()
    recognizer.read("./recognizers/face-trainner.yml")

    labels = {"person_name": 1}
    with open("pickles/face-labels.pickle", 'rb') as f:
        og_labels = pickle.load(f)
        labels = {v:k for k,v in og_labels.items()}

    cap = cv2.VideoCapture(0)
    while detect_state:
        # Capture frame-by-frame
        ret, frame = cap.read()
        gray  = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = face_cascade.detectMultiScale(gray, scaleFactor=1.5, minNeighbors=5)
        for (x, y, w, h) in faces:
            #print(x,y,w,h)
            roi_gray = gray[y:y+h, x:x+w] #(ycord_start, ycord_end)
            roi_color = frame[y:y+h, x:x+w]

            # recognize? deep learned model predict keras tensorflow pytorch scikit learn
            id_, conf = recognizer.predict(roi_gray)
            if conf >=4 and conf <= 85:
                #print(5: #id_)
                #print(labels[id_])
                font = cv2.FONT_HERSHEY_SIMPLEX
                name = labels[id_]
                color = (255, 255, 255)
                stroke = 2
                cv2.putText(frame, name, (x,y), font, 1, color, stroke, cv2.LINE_AA)

                if name == "obama":
                    mic_status = 1
                    
                else:
                    mic_status = 0

            img_item = "7.png"
            cv2.imwrite(img_item, roi_color)

            color = (255, 0, 0) #BGR 0-255 
            stroke = 2
            end_cord_x = x + w
            end_cord_y = y + h
            cv2.rectangle(frame, (x, y), (end_cord_x, end_cord_y), color, stroke)
            #subitems = smile_cascade.detectMultiScale(roi_gray)
            #for (ex,ey,ew,eh) in subitems:
            #	cv2.rectangle(roi_color,(ex,ey),(ex+ew,ey+eh),(0,255,0),2)
        # Display the resulting frame
        cv2.imshow('frame',frame)
        if cv2.waitKey(20) & 0xFF == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()

status = False

@app.route('/')
def hello():
    return "hello world"

@app.route('/<onoff>')
def ledonoff(onoff):
    global autoCDS_status
    global autoFan_status
    global auto_status
    if mic_status == 1:
        global status
        if onoff == "on":
            print("LED Turn on")
            status = True
            GPIO.output([4,5,14,15],1)
            return "LED on"
        elif onoff == "off":
            print("LED Turn off")
            status = False
            GPIO.output([4,5,14,15],0)
            return "LED off"
        elif onoff == "party":
            if status == True:
                print("LED Party Mode")
                for _ in range(1, 11):
                    GPIO.output([4,5,14,15],0)
                    sleep(0.5)
                    GPIO.output([4,5,14,15],1)
                    sleep(0.5)
                return "LED Party Mode"
        elif onoff == "one_on":
            print("LED number 1 Turn on")
            GPIO.output(4,1)
            return "LED number 1 Turn on"
        elif onoff == "two_on":
            print("LED number 2 Turn on")
            GPIO.output(5,1)
            return "LED number 2 Turn on"
        elif onoff == "three_on":
            print("LED number 3 Turn on")
            GPIO.output(15,1)
            return "LED number 3 Turn on"
        elif onoff == "four_on":
            print("LED number 4 Turn on")
            GPIO.output(14,1)
            return "LED number 4 Turn on"
        elif onoff == "one_off":
            print("LED number 1 Turn off")
            GPIO.output(4,0)
            return "LED number 1 Turn off"
        elif onoff == "two_off":
            print("LED number 2 Turn off")
            GPIO.output(5,0)
            return "LED number 2 Turn off"
        elif onoff == "three_off":
            print("LED number 3 Turn off")
            GPIO.output(15,0)
            return "LED number 3 Turn off"
        elif onoff == "four_off":
            print("LED number 4 Turn off")
            GPIO.output(14,0)
            return "LED number 4 Turn off"
        elif onoff == "one_sec":
            GPIO.output(18,1)
            GPIO.output(27,0)
            sleep(1.0)
            GPIO.output(18,0)
            GPIO.output(27,0)
            return "FAN 1 sec on"
        elif onoff == "two_sec":
            GPIO.output(18,1)
            GPIO.output(27,0)
            sleep(2.0)
            GPIO.output(18,0)
            GPIO.output(27,0)
            return "FAN 2 sec on"
        elif onoff == "three_sec":
            GPIO.output(18,1)
            GPIO.output(27,0)
            sleep(3.0)
            GPIO.output(18,0)
            GPIO.output(27,0)
            return "FAN 3 sec on"
        elif onoff == "buzzer_on":
            playBuzzer(melodyList, noteDurations)
            return "turn on buzzer"
        elif onoff == "show_CDS":
            for _ in range(3):
                readVal = readSensor(CDS_CHANNEL)
                voltage = readVal * 4.096 / 4096
                CDS_line = "CDS Val: " + str(readVal) 
                CDS_line2 = "voltage: " + str(voltage)
                displayText(CDS_line, 0,0)
                displayText(CDS_line2, 0,1)
                sleep(1)
            clearTextlcd()
            return "show CDS value"
        elif onoff == "air_cond":
            for _ in range(3):
                try:
                    temperature_c = dhtDevice.temperature
                    humidity = dhtDevice.humidity
                    DHT_line = "Temp: " + str(temperature_c) 
                    DHT_line2 = "Humidity: " + str(humidity)
                    displayText(DHT_line, 0,0)
                    displayText(DHT_line2, 0,1)
                    sleep(1)
                except Exception:
                    sleep(0.01)
            clearTextlcd()
            return "show air condition"
        elif onoff == "motion_detect":
            global pirState
            for _ in range(6):
                if pirState == 0:
                    PIR_line = 'Motion Detect'
                else:
                    PIR_line = 'Motion Ended'
                displayText(PIR_line, 0,0)
                sleep(0.5)
            clearTextlcd()
            return "show motion detect"
        elif onoff == "show_GAS":
            for _ in range(3):
                readVal = readSensor(1)
                voltage = readVal * 4.096 / 4096
                GAS_line = "GAS Val: " + str(readVal) 
                GAS_line2 = "voltage: " + str(voltage)
                displayText(GAS_line, 0,0)
                displayText(GAS_line2, 0,1)
                sleep(1)
            clearTextlcd()
            return "show GAS value"
        elif onoff == "autoLED_on":
            autoCDS_status = True
            AutoCDS()
            return "LED automode on"
        elif onoff == "autoFan_on":
            autoFan_status = True
            AutoFan()
            return "Fan automode on"
        elif onoff == "autoLED_off":
            autoCDS_status = False
            GPIO.output([4, 5, 14, 15], GPIO.LOW)
            return "LED automode off"
        elif onoff == "autoFan_off":
            autoFan_status = False
            offFan()
            return "Fan automode off"
        elif onoff == "automode_on":
            auto_status = True
            AutoMode()
            return "turn on automode"
        elif onoff == "automode_off":
            auto_status = False
            GPIO.output([4, 5, 14, 15], GPIO.LOW)
            offFan()
            return "turn off automode"
    else:
        if onoff == "autoLED_on":
            autoCDS_status = True
            AutoCDS2()
            return "LED automode on"
        elif onoff == "autoFan_on":
            autoFan_status = True
            AutoFan2()
            return "Fan automode on"
        elif onoff == "autoLED_off":
            autoCDS_status = False
            GPIO.output([4, 5, 14, 15], GPIO.LOW)
            return "LED automode off"
        elif onoff == "autoFan_off":
            autoFan_status = False
            offFan()
            return "Fan automode off"
        elif onoff == "automode_on":
            auto_status = True
            AutoMode2()
            return "turn on automode"
        elif onoff == "automode_off":
            auto_status = False
            GPIO.output([4, 5, 14, 15], GPIO.LOW)
            offFan()
            return "turn off automode"
        
            

def main():
    initTextlcd()
    initKeypad()
    initMcp3208()
    global threading
    detect_state = True
    detect_state2 = True
    detect_state3 = True
    t = threading.Thread(target=controlDevice, args=(detect_state,))
    t.daemon = True
    t.start()
    p = threading.Thread(target=readPir, args=(detect_state2,))
    p.daemon = True
    p.start()
    k = threading.Thread(target=controlKeypad, args=(detect_state3,))
    k.daemon = True
    k.start()
    app.run(host='0.0.0.0', port=5000, debug=False)
    
if __name__ == "__main__":
    main()