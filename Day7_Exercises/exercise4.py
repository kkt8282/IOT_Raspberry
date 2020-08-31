import numpy as np
import cv2
import pickle
from time import sleep

import RPi.GPIO as GPIO
import threading
from flask import Flask
app = Flask(__name__)

detect_state = False
mic_status = 0
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

# When everything done, release the capture

def controlDevice(detect_state):
    global mic_status
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

@app.route('/detect_onoff/<onoff>')
def ledonoff(onoff):
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



if __name__ == "__main__":
    GPIO.setmode(GPIO.BCM)
    GPIO.setup([4,5,14,15,18,27],GPIO.OUT,initial=GPIO.LOW)
    GPIO.setwarnings(False)

    global threading
    detect_state = True
    t = threading.Thread(target=controlDevice, args=(detect_state,))
    t.daemon = True
    t.start()
    app.run(host='0.0.0.0', port=5000, debug=False)
