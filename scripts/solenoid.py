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
solenoid_pin = 18

#Set the pin as an output
GPIO.setup(solenoid_pin, GPIO.OUT)



#Set servo to 90 degrees as it's starting position
p = GPIO.PWM(solenoid_pin, 400)


def solenoid ():
	dc = input("To start, press any")
	if dc:  
		print("Starting...")
		p.start
		time.sleep(4)
		print("Stopping...")
		p.stop
  		

try:
	while True:
		solenoid()
	
except KeyboardInterrupt:
	p.stop()
	GPIO.cleanup()
