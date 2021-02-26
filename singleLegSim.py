import pybullet as p
import time
import pybullet_data
from pprint import pprint  
import math 
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
hexapod = p.loadURDF("singleleg.urdf",startPos, startOrientation,globalScaling=3,useFixedBase=True) #load robot and make it bigger
p.setRealTimeSimulation(1) #use system clock to step simulaton
    
p.resetDebugVisualizerCamera(3.2, 109,-11.0,p.getBasePositionAndOrientation(hexapod)[0])
start = time.time()
cpg = CPG()

tol = 0.05
torque = []
for i in range(1000):
    hipPos,shoulderPos,hipdPos,shoulderdPos = cpg.get_motor_commands(start)
    
    p.setJointMotorControl2(hexapod,1,p.POSITION_CONTROL,shoulderPos)
    p.setJointMotorControl2(hexapod,0,p.POSITION_CONTROL,hipPos)
    torque.append(p.getJointState(hexapod,0)[3])
    while 1:

        #print(torque)
        current_pos = p.getJointState(hexapod,1)[0]
        if abs(current_pos - shoulderPos) < tol:
            break
    time.sleep(1./240.)

p.disconnect()

plt.plot(torque)
plt.show()