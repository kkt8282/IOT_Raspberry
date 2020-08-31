import numpy as np
import cv2
import pickle
import board
import digitalio
import adafruit_character_lcd.character_lcd as character_lcd
import RPi.GPIO as GPIO
import GPIO_EX
from time import sleep
import threading

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

detect_state = False
led_status = 0

def controlKeypad(detect_state):
	while detect_state:
		if (led_status == 1):
			try:
				password_1 = []
				password_2 = []
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
							break

			except KeyboardInterrupt:
				GPIO.cleanup()
		else:
			line = 'ACCESS DENIED'
			lcd.clear()
			displayText(line, 0,0)
			sleep(3)
			lcd.clear()

global threading
detect_state = True
t = threading.Thread(target=controlKeypad, args=(detect_state,))
t.daemon = True
t.start()

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

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(4, GPIO.OUT, initial=GPIO.LOW)


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



initTextlcd()
print("start textlcd program...")

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

initKeypad()
print("setup keypad pin")

while(True):
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
				led_status = 1
			else:
				led_status = 0

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
	GPIO.setup(4, GPIO.OUT, initial=GPIO.LOW)
	# Display the resulting frame
	cv2.imshow('frame',frame)
	if cv2.waitKey(20) & 0xFF == ord('q'):
		break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
