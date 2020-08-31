import RPi.GPIO as GPIO
from time import sleep

from flask import Flask
app = Flask(__name__)

status = False

@app.route('/')
def hello():
    return "hello world"

@app.route('/led/<onoff>')
def ledonoff(onoff):
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


if __name__ == "__main__":
    GPIO.setmode(GPIO.BCM)
    GPIO.setup([4,5,14,15],GPIO.OUT,initial=GPIO.LOW)
    app.run(host='0.0.0.0', port=5000, debug=True)
