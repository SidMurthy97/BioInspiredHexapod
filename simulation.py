import pybullet as p
import time
import pybullet_data
from pprint import pprint  
import math 


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

hips = [0,3,6,9,12,15]
shoulder = [1,4,7,10,13,16]

for i in range (1000):
    index = i%len(states)
    pos = states[index]
    print("targert: ",pos)
    p.setJointMotorControlArray(hexapod,shoulder,p.POSITION_CONTROL,[pos]*len(shoulder))
    
    while 1:
        current_pos = p.getJointState(hexapod,1)[0]
        print(current_pos)
        if abs(current_pos - pos) < tol:
            break
    
        
    
    print("-------------------------")
    time.sleep(1./240.)
    

p.disconnect()
