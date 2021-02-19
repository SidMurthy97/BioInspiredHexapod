import pybullet as p
import time
import pybullet_data
from pprint import pprint  
import math 
from RT_CPG_units import get_motor_commands
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
hexapod = p.loadURDF("C:\\Users\\user\\Documents\\year 5\\FYP\\robot_dart\\robots\\pexod.urdf",startPos, startOrientation,globalScaling=3) #load robot and make it bigger
p.setRealTimeSimulation(1) #use system clock to step simulaton

nJoints = p.getNumJoints(hexapod)
states = [0,math.pi/4]

tol = 0.05 #set tolerance 

shoulder1 = [10,13,4]
hips1 = [9,12,3]

shoulder2 = [7,16,1]
hips2 = [6,15,0]

hipbuffer= []
shoulderbuffer= []

start = time.time()
for i in range (1000):
    
    hipPos,shoulderPos,hipdPos,shoulderdPos = get_motor_commands(start)
    
    p.setJointMotorControlArray(hexapod,shoulder1,p.POSITION_CONTROL,[shoulderPos]*len(shoulder1))
    p.setJointMotorControlArray(hexapod,hips1,p.POSITION_CONTROL,[-hipPos,hipPos,-hipPos])

    p.setJointMotorControlArray(hexapod,shoulder2,p.POSITION_CONTROL,[shoulderdPos]*len(shoulder1))
    p.setJointMotorControlArray(hexapod,hips2,p.POSITION_CONTROL,[hipdPos,-hipdPos,hipdPos])
    
    while 1:
        current_pos = p.getJointState(hexapod,10)[0]
        #print(current_pos)
        if abs(current_pos - shoulderPos) < tol:
            break
    #print(p.getBasePositionAndOrientation(hexapod)[0])
    p.resetDebugVisualizerCamera(5,230,200,p.getBasePositionAndOrientation(hexapod)[0])
    time.sleep(1./240.)
    
p.disconnect()
