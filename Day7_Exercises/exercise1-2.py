import RPi.GPIO as GPIO
from time import sleep

from flask import Flask
app = Flask(__name__)

@app.route('/')
def hello():
    return "hello world"

@app.route('/<onoff>')
def ledonoff(onoff):
    if onoff == "one_on":
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


if __name__ == "__main__":
    GPIO.setmode(GPIO.BCM)
    GPIO.setup([4,5,14,15],GPIO.OUT,initial=GPIO.LOW)
    app.run(host='0.0.0.0', port=5000, debug=True)

# http://a555007a338a.ngrok.io