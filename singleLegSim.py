import pybullet as p
import time
import pybullet_data
from pprint import pprint  
import math 
import numpy as np
from RT_CPG_units import CPG
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
from tqdm import trange
from collections import deque

def tripod():
    #initialise CPGs
    for i in range(nLegs):
            cpgUnits.append(CPG(start,0.5))

    #tripod gait allows for bidirectional coupling 
    ncpgs = nLegs
    phase = math.pi

    if nLegs > 1:
        for i in range(ncpgs):
            prevUnit = cpgUnits[(i-1)%ncpgs]
            nextUnit = cpgUnits[(i+1)%ncpgs]
            cpgUnits[i].coupledCPG = [[prevUnit,phase],[nextUnit,phase]]

def plotData(datas):
    for series in datas:
        plt.figure()
        plt.plot(series)
        plt.xlabel("index")

#set pybullet terms 
physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) 
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf") #load plane
startPos = [0,0,1]
startOrientation = p.getQuaternionFromEuler([0,0,0])
hexapod = p.loadURDF("C:\\Users\\murth\\Documents\\year 5\\FYP\\src\\robots\\singleleg.urdf",startPos, startOrientation,globalScaling=3,useFixedBase=True) #load robot and make it bigger
#obstacle = p.loadURDF("C:\\Users\\user\\Documents\\year 5\\FYP\\src\\robots\\box.urdf",[-1,0.8,0], startOrientation,globalScaling=5)
#obstacle = p.loadURDF("duck_vhacd.urdf",[-1,0.8,0], startOrientation,globalScaling=5)
p.setRealTimeSimulation(1) #use system clock to step simulaton
    
p.resetDebugVisualizerCamera(3.2, 178,-1.0,p.getBasePositionAndOrientation(hexapod)[0])
start = time.time()

tol = 0.05
torqueList = []

#printJointInfo(2,hexapod)
#joint indices 

hips = [0]
knees = [1]
ankles = [2]

nLegs = len(hips)

hipPos = np.zeros(nLegs)
kneePos = np.zeros(nLegs)
anklePos = np.zeros(nLegs)

cpgUnits = []
#matrix transform to reverse one side of the robot  
transform = [-1]

start = time.time()
transient = 0
tripod()

#debug outputs
torqueList = []
xtest = []
ytest = []

#p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4,"ripple_gait.mp4")

try:
    for i in trange(1000):
        while time.time() - start < transient:
            #allow CPG units to run for transient period 
            for j in range(nLegs):
                hipPos[j],kneePos[j] = cpgUnits[j].get_motor_commands(start)

        #get positions for all legs 
        for j in range(nLegs):
            hipPos[j],kneePos[j] = cpgUnits[j].get_motor_commands(start)
            anklePos[j] = kneePos[j] #this ensures end effector is perpendicular to ground and allows stability     

        p.setJointMotorControlArray(hexapod,knees,p.POSITION_CONTROL,kneePos)
        p.setJointMotorControlArray(hexapod,hips,p.POSITION_CONTROL,hipPos*transform)
        p.setJointMotorControlArray(hexapod,ankles,p.POSITION_CONTROL,anklePos)

        #add a torque perturbation every 500 iterations
        if i%300 == 0 and i > 0 and cpgUnits[0].x > 0:
            cpgUnits[0].torqueFeedback = 5
            print("purturbation")
        else:
            cpgUnits[0].torqueFeedback = abs(p.getJointState(hexapod,hips[0])[-1])
        
        torqueList.append(cpgUnits[0].offset)
        xtest.append(cpgUnits[0].x)
        ytest.append(cpgUnits[0].y)
        #p.resetDebugVisualizerCamera(5, 50,-35.0,p.getBasePositionAndOrientation(hexapod)[0])
        time.sleep(1./240.)

    p.disconnect()

    #live plotting to see the convergence properties     
    x = deque(maxlen=50)
    y = deque(maxlen=50)
    plt.figure()

    for j in trange(len(xtest)):
        x.append(xtest[j])
        y.append(ytest[j])
        plt.plot(x,y)
        plt.show(block = False)
        plt.pause(0.005)

except KeyboardInterrupt:

    plotData([xtest,ytest,torqueList])
    p.disconnect()