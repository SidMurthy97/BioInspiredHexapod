import pybullet as p
import time
import pybullet_data
from pprint import pprint  
import math 
import numpy as np
from RT_CPG_units import CPG
import matplotlib.pyplot as plt

def printJointInfo():
    for i in range(nJoints):
        print(p.getJointInfo(hexapod,i)[0:2])

#set pybullet terms 
physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) 
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf") #load plane
startPos = [0,0,1]
startOrientation = p.getQuaternionFromEuler([0,0,0])
hexapod = p.loadURDF("C:\\Users\\murth\\Documents\\year 5\\FYP\\src\\robots\\pexod.urdf",startPos, startOrientation,globalScaling=3) #load robot and make it bigger
p.setRealTimeSimulation(1) #use system clock to step simulaton

nJoints = p.getNumJoints(hexapod)

cpgUnits = []

tol = 0.05
hips = [9,15,3,6,12,0]
knees = [10,16,4,7,13,1]
ankles = [11,17,5,8,14,2]

# hips = [9,15]
# knees = [10,16]
# ankles = [11,17]

nLegs = len(hips)

hipPos = np.zeros(nLegs)
kneePos = np.zeros(nLegs)
anklePos = np.zeros(nLegs)

start = time.time()
#gait matrices
tripod = [-1,-1,-1,1,1,1]
tripodPhase = math.pi
transientPeriod = 10


#couple all cpgs to the first one
for i in range(nLegs):
    if i == 0:
        cpgUnits.append(CPG(start,None))
    else:
        cpgUnits.append(CPG(start,cpgUnits[i-1]))
    
# p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4,"tripod_gait.mp4")

for i in range (10000):
    
    # #allow CPG outputs to settle
    # while time.time() - start < transientPeriod:
        
    #     for j in range(nLegs):
    #         hipPos[j],kneePos[j] = cpgUnits[j].get_motor_commands(start)
    #         anklePos[j] = kneePos[j] #this ensures end effector is perpendicular to ground and allows stability      

    for j in range(nLegs):
        hipPos[j],kneePos[j] = cpgUnits[j].get_motor_commands(start)
        anklePos[j] = kneePos[j] #this ensures end effector is perpendicular to ground and allows stability     

    p.setJointMotorControlArray(hexapod,knees,p.POSITION_CONTROL,kneePos)
    p.setJointMotorControlArray(hexapod,hips,p.POSITION_CONTROL,hipPos*tripod)
    p.setJointMotorControlArray(hexapod,ankles,p.POSITION_CONTROL,anklePos)
    
    # while 1:
    #     current_pos = p.getJointState(hexapod,10)[0]
    #     if abs(current_pos - kneePos[0]) < tol:
    #         break
    p.resetDebugVisualizerCamera(5, 50,-35.0,p.getBasePositionAndOrientation(hexapod)[0])
    time.sleep(1./240.)
    
p.disconnect()
