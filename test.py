ult1=ev3.UltrasonicSensor('in1') #Front-sensor
ult2=ev3.UltrasonicSensor('in2') #Right-sensor
ult3=ev3.UltrasonicSensor('in3') #Back-sensor 
ult4=ev3.UltrasonicSensor('in4') #Left-sensoim
import getch                                  
from statistics import mean, pvariance, mode, 
from collections import Counter               
import time
import re
#from client import *                         
                                              
m1=ev3.LargeMotor('outA')   # front           
m2=ev3.LargeMotor('outB')   # right           
m3=ev3.LargeMotor('outC')   # back            
m4=ev3.LargeMotor('outD')   # left            
                                              
                                              
ult1=ev3.UltrasonicSensor('in1') #Front-sensor
ult2=ev3.UltrasonicSensor('in2') #Right-sensor
ult3=ev3.UltrasonicSensor('in3') #Back-sensor 
ult4=ev3.UltrasonicSensor('in4') #Left-sensor

def main:
    comm = input("Ready to test")
    while True:
        if m == re.match(comm, 'Motor ([0..9]*) ([0..9]*) ([0..9]*)')  
