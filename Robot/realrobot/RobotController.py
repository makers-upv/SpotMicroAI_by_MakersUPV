import time
from math import sin, pi
from utilities.kinematics import Kinematic
import numpy as np

#Input parameters
stanceTime = 1000 
swingTime = 500
maxStanceDist = 2

#Internal variables
contact = [[1,1,1,1], 
           [1,0,0,1], 
           [1,1,1,1], 
           [0,1,1,0]] 
t0=0
P = [[0,0,0],[0,0,0]]

#Controller input
move = 1
yStanceDist = 150

control = Kinematic

while True:
    for i in range(2):
        if move:
            if t0 == 0:
                t0 = time.time()*1000
            dt = (time.time()*1000-t0 + i*(stanceTime+swingTime)/2)%(stanceTime+swingTime) 
            
            if dt >= 0 and dt < stanceTime:
                P[i][0]=(stanceTime/2-dt)*yStanceDist/stanceTime
                P[i][1]=0
                
            if dt >= stanceTime and dt < swingTime+stanceTime:
                P[i][0]=-yStanceDist/2 + (dt-stanceTime)*yStanceDist/swingTime
                P[i][1]=yStanceDist/2.3*sin((dt-stanceTime)*pi/swingTime)
    print(P)
    Lp=np.array([[P[0][0],P[0][1],P[0][2],0],[P[1][0],P[1][1],P[1][2],0],[P[1][0],P[1][1],P[1][2],0],[P[0][0],P[0][1],P[0][2],0]])
    Kinematic().setRobot(Lp,(0,0,0),(0,0,0))


"""
    if dt >= 0 and dt < ((stanceTime-swingTime)/2):
        P[0]=-(dt+swingTime/2)*yStanceDist/stanceTime

    if dt >= ((stanceTime-swingTime)/2) and dt < ((stanceTime+swingTime)/2):
        P[0]=(dt-stanceTime/2)*yStanceDist/swingTime
        
    if dt >= ((stanceTime+swingTime)/2) and dt < stanceTime:
        
    if dt >= stanceTime and dt < (stanceTime+swingTime):
"""
