import RPi.GPIO as GPIO
import sys
import os.path
sys.path.append(os.path.join(os.path.dirname(__file__),'..')) # this is done for the AMG88xx folder (you may have to rewrite this to include the path of your AMG file)
from Adafruit_AMG88xx import Adafruit_AMG88xx
from time import sleep
import time
import matplotlib as mpl
mpl.use('tkagg') # to enable real-time plotting in Raspberry Pi
import matplotlib.pyplot as plt
import numpy as np
from pprint import pprint
import Stepper_Test as stepper
import shoot1 as shooter
import shooter_talker as st
import rclpy 

detected = False
rclpy.init(args=None)
shootertk = st.Shooter("Stop moving")


#try:
def main_temp():
    sensor = Adafruit_AMG88xx()

    sleep(0.1)
    global detected

# preallocating variables
    norm_pix = []
    cal_vec = []
    kk = 0
    cal_size = 10 # size of calibration
    cal_pix = []
    time_prev = time.time() # time for analyzing time between plot updates
    
    shooting_threshold = 45
    detected_threshold = 35  #Threshold temperature (when a value is significantly higher than the rest)
    plt.ion()
    max_temp = 0
    for x in range(5):
                # calibration procedure #
        if kk==0:
            print("Sensor should have clear path to calibrate against environment")
                   #     graph = plt.imshow(np.reshape(np.repeat(0,64),(8,8)),cmap=plt.cm.hot,interpolation='lanczos')
                    #    plt.colorbar()
                     #   plt.clim(1,8) # can set these limits to desired range or min/max of current sensor reading
                     #   plt.draw()
        norm_pix = sensor.readPixels() # read pixels
        
        print(np.reshape(norm_pix,(8,8)))
            
        max_temp = np.amax(norm_pix) #Finding max temperature in the array
        
        index = np.where(norm_pix==max_temp) #Finding the array index
        print("Array is:",index)
        print("First element of Array:",index[0][0])
        index = np.int(index[0][0])
        range_left,range_right,range_bottom = np.array([0,1,2,8,9,10,16,17,18,24,25,26,32,33,34,40,41,42,48,49,50,56,57,58]),np.array([5,6,7,13,14,15,21,22,23,29,30,31,37,38,39,45,46,47,53,54,55,61,62,63]),np.array([3,4,11,12]) #Making ranges 
                
        print(index)
        print(max_temp)
        range_acceptable = np.array([18,19,20,21,26,27,28,29]) #Smack Center of Array
             
        if  max_temp>=detected_threshold:
            if detected == False: 
                rclpy.spin_once(shootertk)
                detected = True
          
            if np.any(index in range_acceptable):
                print("In the middle/ On target")
                shootertk.string1 = "stop"
                rclpy.spin_once(shootertk)

                if max_temp <= shooting_threshold:
                	shootertk.string1 = "forward"
                	rclpy.spin_once(shootertk)
                	break
                else:
                    shootertk.string1 = "stop"
                    rclpy.spin_once(shootertk)
                    shooter.main()
                    shootertk.string1 = "Continue moving"
                    rclpy.spin_once(shootertk)
                    shootertk.destroy_node()
                    rclpy.shutdown()
                    return "Finished Shooting"


            elif np.any(index in range_bottom):
                shootertk.string1 = "stop"
                rclpy.spin_once(shootertk)
                print("Aiming Higher")
                stepper.step(-2)  
                
            elif np.any(index in range_left):
                print("Aiming more to Left")
                shootertk.string1 = "left"
                rclpy.spin_once(shootertk)
                
            elif np.any(index in range_right):
                print("Aiming more to Right")
                shootertk.string1 = "right"
                rclpy.spin_once(shootertk)
                
            elif not(index):
                print("Empty Index")
                continue
            
            else:
                if max_temp <= shooting_threshold:
                    shootertk.string1 = "forward"
                    rclpy.spin_once(shootertk)
                    break
                print("Slightly High But Shooting")
                shootertk.string1 = "stop"
                rclpy.spin_once(shootertk)
                shooter.main()  
                shootertk.string1 = "Continue moving"
                rclpy.spin_once(shootertk)
                return "Finished Shooting"
                #shooter.main()
        time.sleep(1) #Delay for 2 seconds
        cal_pix = [] 
       
while(1):
    condition = main_temp()
    if condition == "Finished Shooting":
        print("Finished Shooting!!!")
        #GPIO.cleanup()
        break
