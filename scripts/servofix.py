#EG2310
#This code is used to show how electronic hardware can be programmed
#Relevant rpi.gpio libraries might need to be installed if not already done

import time
import RPi.GPIO as GPIO

#set pin numbering convention
#Can't use both,please choose one and comment away the other
GPIO.setmode(GPIO.BCM)
#GPIO.setmode(GPIO.BOARD)

#Choose an appropriate pwm channel to be used to control the servo
servo_pin = 18

#Set the pin as an output
GPIO.setup(servo_pin, GPIO.OUT)

#Initialise the servo to be controlled by pwm with 50 Hz frequency
p = GPIO.PWM(servo_pin, 50)

#Set servo to 90 degrees as it's starting position
p.start(2.5)

def turnin ():
      #  try:
               # dc =int(input ('Enter degree: '))
       # if  0<=dc<=190:
            #angle = (dc/18)+2.5
   angle = 5
   p.ChangeDutyCycle(angle)
     #   except ValueError:
    #            print("Enter Again")


