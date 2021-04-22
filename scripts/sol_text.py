#Solenoid
import RPi.GPIO as GPIO
import time




#set pin numbering convention
#Can't use both,please choose one and comment away the other
GPIO.setmode(GPIO.BCM)
#GPIO.setmode(GPIO.BOARD)

#Set the pin to be tested and programmed
test_point = 21

#Set the pin as an output
GPIO.setup(test_point, GPIO.OUT)
def call():
#Loop forever
	GPIO.output(test_point, GPIO.HIGH) 
	time.sleep(1)
	GPIO.output(test_point,GPIO.LOW)
	time.sleep(1)

#except KeyboardInterrupt:

#    	GPIO.cleanup()
    
