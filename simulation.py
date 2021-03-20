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
#hexapod = p.loadURDF("C:\\Users\\murth\\Documents\\year 5\\FYP\\src\\robots\\stationaryHexapod.urdf",startPos, startOrientation,globalScaling=3,useFixedBase=True) #load robot and make it bigger
hexapod = p.loadURDF("C:\\Users\\murth\\Documents\\year 5\\FYP\\src\\robots\\pexod.urdf",startPos, startOrientation,globalScaling=3) #load robot and make it bigger
p.setRealTimeSimulation(1) #use system clock to step simulaton

nJoints = p.getNumJoints(hexapod)

cpgUnits = []

tol = 0.05

#legs are defined in an anticlockwise way
hips = [9,15,3,0,12,6]
knees = [10,16,4,1,13,7]
ankles = [11,17,5,2,14,8]

# hips = [9,15]
# knees = [10,16]
# ankles = [11,17]

nLegs = len(hips)

hipPos = np.zeros(nLegs)
kneePos = np.zeros(nLegs)
anklePos = np.zeros(nLegs)

start = time.time()

#matrix transform to reverse one side of the robot  
tripod = [-1,-1,-1,1,1,1]



#initialise CPGs
for i in range(nLegs):
        cpgUnits.append(CPG(start))


gait = 'm' #t:tripod, m:metachronal


#set up CPG framework for different gaits
if gait == 't':
    #tripod gait allows for bidirectional coupling 
    ncpgs = nLegs
    phase = math.pi
    for i in range(ncpgs):
        prevUnit = cpgUnits[(i-1)%ncpgs]
        nextUnit = cpgUnits[(i+1)%ncpgs]
        cpgUnits[i].coupledCPG = [[prevUnit,phase],[nextUnit,phase]]

elif gait == 'm':
    #framework from campos et al
    cpgUnits[5].coupledCPG = [[cpgUnits[0],math.pi]]
    cpgUnits[1].coupledCPG = [[cpgUnits[0],math.pi/3]]
    cpgUnits[4].coupledCPG = [[cpgUnits[5],math.pi/3],[cpgUnits[1],math.pi]]
    cpgUnits[2].coupledCPG = [[cpgUnits[1],math.pi/3]]
    cpgUnits[3].coupledCPG = [[cpgUnits[2],math.pi],[cpgUnits[4],math.pi/3]]


# p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4,"tripod_gait.mp4")
for i in range (10000):

    #get positions for all legs 
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
    #p.resetDebugVisualizerCamera(5, 50,-35.0,p.getBasePositionAndOrientation(hexapod)[0])
    time.sleep(1./240.)
    
p.disconnect()
