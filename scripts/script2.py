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

#INitialise the pwm obj with 1kz frequency
pwm = GPIO.PWM(test_point,1000)
pwm.start(0)

#Begin pwm experiment
print("Start")

for i in range(0,100,1):
        pwm.ChangeDutyCycle(i)
        print("Brightness is ",i,"%")
        time.sleep(0.2)
else:
        print("Finished")
#End the script and exit
pwm.stop()
GPIO.cleanup()







