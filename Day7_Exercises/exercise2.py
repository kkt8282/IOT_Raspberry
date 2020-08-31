import RPi.GPIO as GPIO
from time import sleep

from flask import Flask
app = Flask(__name__)

@app.route('/fan/<seconoff>')
def fanseconoff(seconoff):
    if seconoff == "one_sec":
        GPIO.output(18,1)
        GPIO.output(27,0)
        sleep(1.0)
        GPIO.output(18,0)
        GPIO.output(27,0)
        return "FAN 1 sec on"
    elif seconoff == "two_sec":
        GPIO.output(18,1)
        GPIO.output(27,0)
        sleep(2.0)
        GPIO.output(18,0)
        GPIO.output(27,0)
        return "FAN 2 sec on"
    elif seconoff == "three_sec":
        GPIO.output(18,1)
        GPIO.output(27,0)
        sleep(3.0)
        GPIO.output(18,0)
        GPIO.output(27,0)
        return "FAN 3 sec on"

if __name__ == "__main__":
    GPIO.setmode(GPIO.BCM)
    GPIO.setup([18,27],GPIO.OUT,initial=GPIO.LOW)
    app.run(host='0.0.0.0', port=5000, debug=True)
    # http://a555007a338a.ngrok.io