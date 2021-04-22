#EG2310
#This code is used to show how electronic hardware can be programmed
#Relevant rpi.gpio libraries might need to be installed if not already done

import time
import RPi.GPIO as GPIO

#set pin numbering convention
#Can't use both,please choose one and comment away the other
GPIO.setmode(GPIO.BCM)
#GPIO.setmode(GPIO.BOARD)

#Set the pin to be tested and programmed
test_point = 21

#Set the pin as an output
GPIO.setup(test_point, GPIO.OUT)

#Loop forever
try:
        while True:
                GPIO.output(test_point, GPIO.HIGH)
                time.sleep(1)
                GPIO.output(test_point,GPIO.LOW)
                time.sleep(1)
except KeyboardInterrupt:
        GPIO.cleanup()


