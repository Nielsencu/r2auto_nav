import RPi.GPIO as GPIO
#GPIO.setmode(GPIO.BOARD)
import time

GPIO.setmode(GPIO.BCM)
back_shooter = 16
front_shooter = 26
GPIO.setup(front_shooter,GPIO.OUT)
GPIO.setup(back_shooter,GPIO.OUT)
       
def main():
    GPIO.output(front_shooter,GPIO.HIGH)
    time.sleep(5)
    GPIO.output(back_shooter, GPIO.HIGH)
    print("has run")
    time.sleep(7)
    GPIO.output(front_shooter, GPIO.LOW)
    GPIO.output(back_shooter, GPIO.LOW)
    GPIO.cleanup()
    exit

#main()
