import pybullet as p
import time
import pybullet_data
from pprint import pprint  
import math 
import numpy as np
from RT_CPG_units import CPG
import matplotlib.pyplot as plt
from tqdm import trange
from collections import deque

def printJointInfo(bot):
    n = p.getNumJoints(hexapod)
    for i in range(n):
        print(p.getJointInfo(bot,i)[0:2])

#set up CPG framework for different gaits
def tripod():
    #initialise CPGs
    for i in range(nLegs):
            cpgUnits.append(CPG(start,0.5))

    #tripod gait allows for bidirectional coupling 
    ncpgs = nLegs
    phase = math.pi
    for i in range(ncpgs):
        prevUnit = cpgUnits[(i-1)%ncpgs]
        nextUnit = cpgUnits[(i+1)%ncpgs]
        cpgUnits[i].coupledCPG = [[prevUnit,phase],[nextUnit,phase]]

def metachronal():
    #initialise CPGs
    for i in range(nLegs):
            cpgUnits.append(CPG(start,5/6))
    #framework from campos et al
    cpgUnits[0].coupledCPG = [None]
    cpgUnits[1].coupledCPG = [[cpgUnits[0],math.pi/3]]
    cpgUnits[2].coupledCPG = [[cpgUnits[1],math.pi/3]]
    cpgUnits[3].coupledCPG = [[cpgUnits[2],math.pi],[cpgUnits[4],math.pi/3]]
    cpgUnits[4].coupledCPG = [[cpgUnits[5],math.pi/3],[cpgUnits[1],math.pi]]
    cpgUnits[5].coupledCPG = [[cpgUnits[0],math.pi]]

def ripple():
    #initialise CPGs
    for i in range(nLegs):
            cpgUnits.append(CPG(start,3/4))   

    cpgUnits[0].coupledCPG = [None]
    cpgUnits[1].coupledCPG = [[cpgUnits[0],-3*math.pi/2]]
    cpgUnits[2].coupledCPG = [[cpgUnits[1],math.pi/2]]
    cpgUnits[3].coupledCPG = [[cpgUnits[2],math.pi],[cpgUnits[4],math.pi/2]]
    cpgUnits[4].coupledCPG = [[cpgUnits[5],math.pi/2],[cpgUnits[1],math.pi]]
    cpgUnits[5].coupledCPG = [[cpgUnits[0],-math.pi]]

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
#hexapod = p.loadURDF("C:\\Users\\murth\\Documents\\year 5\\FYP\\src\\robots\\stationaryHexapod.urdf",startPos, startOrientation,globalScaling=3,useFixedBase=True) #load robot and make it bigger
hexapod = p.loadURDF("C:\\Users\\murth\\Documents\\year 5\\FYP\\src\\robots\\pexod.urdf",startPos, startOrientation,globalScaling=3) #load robot and make it bigger
p.setRealTimeSimulation(1) #use system clock to step simulaton


tol = 0.05

#legs are defined in an anticlockwise way 
hips = [9,15,3,0,12,6]
knees = [10,16,4,1,13,7]
ankles = [11,17,5,2,14,8]

nLegs = len(hips)

hipPos = np.zeros(nLegs)
kneePos = np.zeros(nLegs)
anklePos = np.zeros(nLegs)
cpgUnits = []


#matrix transform to reverse one side of the robot  
transform = [-1,-1,-1,1,1,1]

start = time.time()
transient = 10
tripod()
#metachronal()
highTorque = False
ankleFactor = [1] * nLegs
perturbation = True
torqueTarget = 5 #specify which cpg is being tested

#debug outputs
torqueList = []
xtest = []
ytest = []

cpg1,cpg2,cpg3 = [],[],[]

#p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4,"ripple_gait.mp4")

try:
    for i in range(10000):
        while time.time() - start < transient:
            #allow CPG units to run for transient period 
            for j in range(nLegs):
                hipPos[j],kneePos[j] = cpgUnits[j].get_motor_commands(start)

        #get positions for all legs 
        for j in range(nLegs):
            hipPos[j],kneePos[j] = cpgUnits[j].get_motor_commands(start)
            anklePos[j] = kneePos[j] * ankleFactor[j] #this ensures end effector is perpendicular to ground and allows stability     

        #if obstacle has not been detected then do the default task
        if not highTorque:
            
            if cpgUnits[torqueTarget].x < 0: #reset to default after torque modulation
                cpgUnits[torqueTarget].hopfA = 20
                ankleFactor[torqueTarget] = 1

            p.setJointMotorControlArray(hexapod,knees,p.POSITION_CONTROL,kneePos)
            p.setJointMotorControlArray(hexapod,hips,p.POSITION_CONTROL,hipPos*transform)
            p.setJointMotorControlArray(hexapod,ankles,p.POSITION_CONTROL,anklePos)
            
            #debug outputs to check limitcxycle 
            xtest.append(cpgUnits[0].x)
            ytest.append(cpgUnits[0].y)

            cpg1.append(cpgUnits[1].x)
            cpg2.append(cpgUnits[2].x)
            cpg3.append(cpgUnits[3].x)

            #debug outputs to check phase 

        
        else: #pause until the cpg comes back around to allow the retraction of the leg 
            #print(criticalHip,criticalKnee)
            if hipPos[torqueTarget] < criticalHip and hipPos[torqueTarget] > criticalHip - 0.25 and kneePos[torqueTarget] < criticalKnee and kneePos[torqueTarget] > criticalKnee - 0.25:
                highTorque = False
        
        #add a torque perturbation every 250 iterations
        if i > 600 and cpgUnits[torqueTarget].x > 0.75 and perturbation == True:
            cpgUnits[torqueTarget].torqueFeedback = 10
            cpgUnits[torqueTarget].hopfA = 1   
            highTorque = True 
            perturbation = False #we only want one perturbation 
            criticalHip = hipPos[torqueTarget]        
            criticalKnee = kneePos[torqueTarget]
            ankleFactor[torqueTarget] = -1 #make ankle open up to step over the obstacle 
            print("Perturbation at i = ",i)            
        
        else:
            for j in range(nLegs):
                #cpgUnits[j].torqueFeedback = abs(p.getJointState(hexapod,hips[j])[-1])
                cpgUnits[j].torqueFeedback = 0
        
        torqueList.append(cpgUnits[0].offset)

        p.resetDebugVisualizerCamera(5, 50,-35.0,p.getBasePositionAndOrientation(hexapod)[0])
        time.sleep(1./240.)

    p.disconnect()

    plt.figure()
    plt.plot(xtest)
    plt.plot(cpg1)
    plt.plot(cpg2)
    plt.plot(cpg3)
    plt.show()
    #live plotting to see the convergence properties     
    # x = deque(maxlen=50)
    # y = deque(maxlen=50)
    # plt.figure()
    # plt.grid()
    # for j in trange(300,len(xtest)):
    #     x.append(xtest[j])
    #     y.append(ytest[j])
    #     plt.plot(x,y)
    #     plt.show(block = False)
    #     plt.pause(0.005)
    # plt.show()

except KeyboardInterrupt:

    plotData([xtest,ytest,torqueList])
    p.disconnect()

