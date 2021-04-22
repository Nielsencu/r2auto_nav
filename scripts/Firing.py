#Solenoid
import RPi.GPIO as GPIO
from time import sleep
from Timer import Timer




#set pin numbering convention
#Can't use both,please choose one and comment away the other
GPIO.setmode(GPIO.BCM)
#GPIO.setmode(GPIO.BOARD)

#Set the pin to be tested and programmed
feed_motor = 12
fire_motor = 13
#Set the pin as an output
GPIO.setup(feed_motor, GPIO.OUT)
GPIO.setup(fire_motor,GPIO.OUT)
condition = True
t=Timer()

try:
    while condition:
         t = Timer(text="You waited {:.1f} seconds")
         t.start()
         GPIO.output(feed_motor, GPIO.HIGH)
         print("Feeding Mechanism Activated")
         GPIO.output(fire_motor,GPIO.HIGH)
         print("Firing Mechanism Activated")
         if t == 30:
             GPIO.output(feed_motor,GPIO.LOW)
             print("Feeding Mechanism De-activated")
             GPIO.output(fire_motor,GPIO.LOW)
             print("Firing Mechanism De-activated")
             t.stop()          
             condition =  False
except KeyboardInterrupt:
     GPIO.cleanup()

