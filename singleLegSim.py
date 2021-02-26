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
hexapod = p.loadURDF("C:\\Users\\user\\Documents\\year 5\\FYP\\src\\robots\\singleleg.urdf",startPos, startOrientation,globalScaling=3,useFixedBase=True) #load robot and make it bigger
obstacle = p.loadURDF("C:\\Users\\user\\Documents\\year 5\\FYP\\src\\robots\\box.urdf",[-1,0.8,0], startOrientation,globalScaling=5)
#obstacle = p.loadURDF("duck_vhacd.urdf",[-1,0.8,0], startOrientation,globalScaling=5)
p.setRealTimeSimulation(0) #use system clock to step simulaton
    
p.resetDebugVisualizerCamera(3.2, 194,-31.0,p.getBasePositionAndOrientation(hexapod)[0])
start = time.time()
cpg = CPG()

tol = 0.05
torqueList = []

#joint indices 
hip,knee = 0,1
for i in range(3000):
    p.stepSimulation()
    hipPos,kneePos,hipdPos,kneedPos = cpg.get_motor_commands(start)
    
    p.setJointMotorControl2(hexapod,knee,p.POSITION_CONTROL,kneePos)
    p.setJointMotorControl2(hexapod,hip,p.POSITION_CONTROL,hipPos)
    p.setJointMotorControl2(hexapod,2,p.POSITION_CONTROL,0)

    torque = abs(p.getJointState(hexapod,hip)[3]*cpg.attenuation)
    torqueList.append(torque)
    cpg.torqueFeedback = torque
    
    obPos,obOrn = p.getBasePositionAndOrientation(obstacle)
    force = [0,0,0]
    p.applyExternalForce(objectUniqueId=obstacle, linkIndex=-1,
                         forceObj=force, posObj=obPos, flags=p.WORLD_FRAME)
    # while 1:
               
    #     #checking only hip for now, but should ideally check all joints 
    #     current_pos = p.getJointState(hexapod,hip)[0]
    #     if abs(current_pos - hipPos) < tol:
    #         break
    time.sleep(1./240.)

p.disconnect()

plt.plot(torqueList)
plt.show()