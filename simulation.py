import pybullet as p
import time
import pybullet_data
physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
hexapod = p.loadURDF("C:\\Users\\user\\Documents\\year 5\\FYP\\robot_dart\\robots\\pexod.urdf")

nJoints = p.getNumJoints(hexapod)

states = [0,0.5]

p.setJointMotorControlArray(hexapod,range(nJoints),p.POSITION_CONTROL,[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0])
#set the center of mass frame (loadURDF sets base link frame) startPos/Ornp.resetBasePositionAndOrientation(boxId, startPos, startOrientation)
for i in range (1000):
    p.stepSimulation()
    p.setJointMotorControlArray(hexapod,range(nJoints),p.POSITION_CONTROL,[0,0,0,0,i%len(states),0,0,0,0,0,0,0,0,0,0,0,0,0])
    time.sleep(1./240.)


cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
print(cubePos,cubeOrn)
p.disconnect()
