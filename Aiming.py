#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Apr  3 01:29:16 2021

@author: fudgebutthead2
"""

#!usr/bin/python
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
import r2_autonav 
import Stepper as step
import Firing as Fire
class Aim_Mechanism():
    def __init__(self):
        self.sensor = Adafruit_AMG88xx()
# wait for AMG to boot
        self.sleep(0.1)

# preallocating variables
        self.norm_pix = []
        self.cal_vec = []
        self.kk = 0
        self.cal_size = 10 # size of calibration
        self.cal_pix = []
        self.time_prev = time.time() # time for analyzing time between plot updates
        self.threshold_temp = 5 #Threshold temperature (when a value is significantly higher than the rest)
        self.plt.ion()
        self.max_temp = 0
        self.rotate = r2auto_nav.AutoNav()

#try:
 #  while(1):
    def cal(self):            # calibration procedure #
        if self.kk==0:
            print("Sensor should have clear path to calibrate against environment")
                   #     graph = plt.imshow(np.reshape(np.repeat(0,64),(8,8)),cmap=plt.cm.hot,interpolation='lanczos')
                    #    plt.colorbar()
                     #   plt.clim(1,8) # can set these limits to desired range or min/max of current sensor reading
                     #   plt.draw()
        self.norm_pix = self.sensor.readPixels() # read pixels
        if self.kk<self.cal_size+1:
            self.kk+=1
        if self.kk==1:
            self.cal_vec = self.norm_pix
            #continue
        elif self.kk<=self.cal_size:           #Normalisation code, meaning environment should not affect
            for xx in range(0,len(self.norm_pix)):
                self.cal_vec[xx]+=self.norm_pix[xx]
                if self.kk==self.cal_size:
                    self.cal_vec[xx] = self.cal_vec[xx]/self.cal_size
                    continue
            else:
                [self.cal_pix.append(self.norm_pix[x]-self.cal_vec[x]) for x in range(0,len(self.norm_pix))]
                if min(self.cal_pix)<0:
                    for y in range(0,len(self.cal_pix)):
                        self.cal_pix[y]+=abs(min(self.cal_pix))
            
            # Moving Pixel Plot #
           # print(np.reshape(self.cal_pix,(8,8))) # this helps view the output to ensure the plot is correct
            #graph.set_data(np.reshape(cal_pix,(8,8))) # updates heat map in 'real-time'
            #plt.draw() # plots updated heat map
    def aim(self):            
            self.max_temp = np.amax(self.cal_pix) #Finding max temperature in the array
            if self.max_temp >= self.threshold_temp:
                index = np.where(self.cal_pix == self.max_temp) #Finding the array index
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
            
                if np.any(index==range_acceptable):
                    print("On Target")
                    Fire()
                elif np.any(index==range_left):
                    print("Aiming more to Left")
                    rotate.rotatebot(-20)
                elif np.any(index==range_right):
                    print("Aiming more to Right")
                    rotate.rotatebot(20)
                elif np.any(index==range_top):
                    print("Aiming Lower")
                    step.clockwise()
                else:
                    print("Aiming Higher")
                    step.anticlockwise()
            time.sleep(2) #Delay for 2 seconds
            self.cal_pix = [] # off-load variable for next reading
           # print(time.time()-time_prev) # prints out time between plot updates
           # time_prev = time.time()
            

            

                
            #cal_pix = [] # off-load variable for next reading
            
            #print(time.time()-time_prev) # prints out time between plot updates
            #time_prev = time.time()
        
#except KeyboardInterrupt:
 #       print("CTRL-C: Program Stopping via Keyboard Interrupt...")

#finally:
 #       print("Exiting Loop")      

 
