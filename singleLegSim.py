import pybullet as p
import time
import pybullet_data
from pprint import pprint  
import math 
from RT_CPG_units import CPG
import matplotlib.pyplot as plt
import numpy as np

def printJointInfo(n,hexapod):
    for i in range(n):
        print(p.getJointInfo(hexapod,i)[0:2])


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
    
p.resetDebugVisualizerCamera(3.2, 194,-31.0,p.getBasePositionAndOrientation(hexapod)[0])
start = time.time()

tol = 0.05
torqueList = []

#printJointInfo(2,hexapod)
#joint indices 

hips = [0,3]
knees = [1,4]
ankles = [2,5]

nLegs = len(hips)

hipPos = np.zeros(nLegs)
kneePos = np.zeros(nLegs)
anklePos = np.zeros(nLegs)

cpgUnits = []
for i in range(nLegs):
    if i == 0:
        cpgUnits.append(CPG(start,None))
    else:
        cpgUnits.append(CPG(start,cpgUnits[i-1]))

for i in range(3000):
#    p.stepSimulation()
 
    for j in range(nLegs):
        hipPos[j],kneePos[j] = cpgUnits[j].get_motor_commands(start)
        anklePos[j] = kneePos[j]

    p.setJointMotorControlArray(hexapod,knees,p.POSITION_CONTROL,kneePos)
    p.setJointMotorControlArray(hexapod,hips,p.POSITION_CONTROL,hipPos)
    p.setJointMotorControlArray(hexapod,ankles,p.POSITION_CONTROL,anklePos)
    
    time.sleep(1./240.)

p.disconnect()
