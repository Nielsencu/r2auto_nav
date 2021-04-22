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
#import Firing as Fire
import shoot1 as shooter


#try:
def main_temp():
    sensor = Adafruit_AMG88xx()
# wait for AMG to boot
    sleep(0.1)

# preallocating variables
    norm_pix = []
    cal_vec = []
    kk = 0
    cal_size = 10 # size of calibration
    cal_pix = []
    time_prev = time.time() # time for analyzing time between plot updates
    threshold_temp =20 #Threshold temperature (when a value is significantly higher than the rest)
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
        if kk<cal_size+1:
            kk+=1
        if kk==1:
            cal_vec = norm_pix
            continue
        elif kk<=cal_size:           #Normalisation code, meaning environment should not affect
            for xx in range(0,len(norm_pix)):
                cal_vec[xx]+=norm_pix[xx]
                if kk==cal_size:
                    cal_vec[xx] = cal_vec[xx]/cal_size
                    continue
            else:
                [cal_pix.append(norm_pix[x]-cal_vec[x]) for x in range(0,len(norm_pix))]
                if min(cal_pix)<0:
                    for y in range(0,len(cal_pix)):
                        cal_pix[y]+=abs(min(cal_pix))
            
            # Moving Pixel Plot #
            print(np.reshape(cal_pix,(8,8))) # this helps view the output to ensure the plot is correct
            #graph.set_data(np.reshape(cal_pix,(8,8))) # updates heat map in 'real-time'
            #plt.draw() # plots updated heat map          
            max_temp = np.amax(cal_pix) #Finding max temperature in the array
            if max_temp >= threshold_temp:
                index = np.where(cal_pix == max_temp) #Finding the array index
                index = index[0]
                range_left,range_right,range_top = np.array([0,1,8,9,16,17,24,25,32,33,40,41,48,49,56,57,64]),np.array([6,7,14,15,22,23,30,31,38,39,46,47,54,55,62,63]),np.array([2,3,4,5,10,11,12,13]) #Making ranges 
                #for i in range(0,9,8):           #Appending range_left
                 #   np.append(i,range_left)
                  #  for j in range(1,9,8):
                   #     np.append(j,range_left)
                #for i in range(6,9,8):           #Appending range_right
                 #   np.append(i,range_right)
                  #  for j in range(7,9,8):
                   #     np.append(j,range_right)
                print(index)
                range_acceptable = np.array([18,19,20,21,26,27,28,29,30]) #Smack Center of Array
               # range_acceptable = np.concatenate(range_acceptable,range_top)
               # for i in range(8):
                   # for j in range(8):
                       # print(cal_pix[i*8+j],sep="")
                   # print("\n")
                
                if np.any(index==range_acceptable):
                    print("On Target")
                    time.sleep(2)
                    shooter.main()
                    return False
    		#      Fire()
                elif np.any(index==range_right):
                    print("Aiming more to Right")
                    #auto_nav.rotate_bot(10)
                elif np.any(index==range_left):
                    print("Aiming more to Left")
                    #auto_nav.rotate_bot(-10)
                elif np.any(index==range_top):
                    print("Aiming Lower")
                    stepper.step(2)
                   
                else:
                    print("Aiming Higher")
                    stepper.step(-2)
                    #time.sleep(2)
                   # shooter.main()
            time.sleep(2) #Delay for 2 seconds
            cal_pix = [] # off-load variable for next reading
       # print(time.time()-time_prev) # prints out time between plot updates
       # time_prev = time.time()
    
#except KeyboardInterrupt:
  #      print("CTRL-C: Program Stopping via Keyboard Interrupt...")
        
#finally:
 #       print("Exiting Loop")    
while(1):
    
    main_temp()
    if main_temp() == False:
       exit

