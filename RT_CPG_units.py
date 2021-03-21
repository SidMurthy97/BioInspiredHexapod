import matplotlib.pyplot as plt
import numpy as np
import math 
import time 
from oscillators import angle_to_position
import pprint


class CPG():

    def __init__(self,prev,beta):
        #hopf parameters
        self.hopfMu = 1

        self.stanceF = ((1 - beta)/beta)*math.pi
        self.swingF = (1)*math.pi
        
        #Torque feedback parameters
        self.torqueFeedback = 0
        self.attenuation = 1/30
        self.offset = 0
        self.n = 0
        self.k = 60
        self.elevator = False
        self.criticalHipPos = 0
        self.criticalKneePos = 0
        self.release = False

        #Numerical solving parameters
        self.prev = prev
        self.x = 1
        self.y = 0

        #phase coupling parameters
        self.coupledCPG = [None]
        self.delta = 1

        #debug outputs
        self.omegaList = []

    def hopfOld(self,t):
        
        #keep count of the number of times this function has been called to make online averaging accurate
        #averages only the last k values
        if self.n < self.k:
            self.n+=1
        else:
            self.n = self.k
        
        mu,omega = self.hopfMu,self.hopfOmega

        #Online averaging to make the effect of large torque last longer.
        self.offset = self.offset + (self.torqueFeedback * self.attenuation - self.offset)/self.n

        if self.torqueFeedback > 50 and t > 1 and self.release == False:
            self.elevator = True
            offset = 0
        else:
            self.release = False
            offset = self.offset
            
        
        
        #add offset only to knee
        x = math.sqrt(mu)*np.cos(omega*t + self.phase) + offset
        y = math.sqrt(mu)*np.sin(omega*t + self.phase)
        
        #self.hopfOmega = math.pi/(math.exp(-50*x) + 1) + (math.pi)/(math.exp(50*x) + 1)

        
        return x,y,self.hopfOmega

    def hopf(self,z):
        x,y = z
        a,mu,b= 20,1,20

        #angular frequency can be modulated using current position 

        omega = self.swingF/(math.exp(-b*x) + 1) + self.stanceF/(math.exp(b*x) + 1)
        
        self.omegaList.append(omega)

        dx = a*(mu - x**2 - y**2)*x - omega*y
        dy =  a*(mu - x**2 - y **2)*y + omega*x
        #print(self.coupledCPG)
        if self.coupledCPG[0] != None:
            
            deltaSum = 0
            #print(self.coupledCPG)
            for couple in self.coupledCPG:
                phase = couple[1]
                unit = couple[0]
                deltaSum +=  (unit.y * np.cos(phase) - unit.x * np.sin(phase))

            return dx, dy + 0.4*deltaSum

            
            # deltaSumX,deltaSumY = 0,0
            # #print(self.coupledCPG)
            # for couple in self.coupledCPG:
            #     phase = couple[1]
            #     unit = couple[0]

            #     deltaSumX = (unit.x + unit.y)/(math.sqrt(unit.x**2 + unit.y**2)) * np.sin(phase)
            #     deltaSumY = (unit.x + unit.y)/(math.sqrt(unit.x**2 + unit.y**2)) * np.cos(phase)
            #     #print(deltaSumX,deltaSumY)
            # return dx - deltaSumX, dy + deltaSumY
            
        else:
            return dx,dy
    
    def euler(self,t,x,y):
        
        #set new timestamp to calculate next step size
        self.prev = time.time()

        #get gradients using the hopf oscillator equations
        dx,dy = self.hopf([x,y])

        #estimate next value using euler method 
        self.x =  x + t*dx
        self.y = y + t*dy

        #return new coordinates
        return self.x,self.y

    def vdp(self,t,z):
        b,w = 1,math.pi
        x,y = z

        x = (2/math.sqrt(b))*np.cos(w*t) 
        y = ((2*w)/math.sqrt(b))*np.sin(w*t) 

        return x,y

    def get_motor_commands(self,start,realWorld = False):

        #use euler method to solve hopf equations     
        x,y= self.euler(time.time() - self.prev,self.x,self.y)
              
        if realWorld: #if running on hardware convert to position
            hip = 30 *y
            knee = 30*x if x > 0 else 0 

            return angle_to_position(hip),angle_to_position(knee)
        
        else: #if running in simulation, give radian angles
            hip = (math.pi/6)*y
            knee = (math.pi/6)*x if x > 0 else 0 

            return hip,knee



if __name__ == "__main__":
    xr,yr,t,omegaList = [],[], [],[]
    
    start = time.time()

    cpgUnits = []
    ncpgs = 6

    #lists to store cpg outputs
    cpg1,cpg2,cpg3,cpg4,cpg5,cpg6 = [],[],[],[],[],[]
    cpg1x,cpg2x = [],[]
    #initialise CPGs
    for i in range(ncpgs):
            cpgUnits.append(CPG(start))
    

    #couple CPGs according to Campos et al
    cpgUnits[0].coupledCPG = [None]
    cpgUnits[1].coupledCPG = [[cpgUnits[0],math.pi/3]]
    cpgUnits[2].coupledCPG = [[cpgUnits[1],math.pi/3]]
    cpgUnits[3].coupledCPG = [[cpgUnits[2],math.pi],[cpgUnits[4],math.pi/3]]
    cpgUnits[4].coupledCPG = [[cpgUnits[5],math.pi/3],[cpgUnits[1],math.pi]]
    cpgUnits[5].coupledCPG = [[cpgUnits[0],math.pi]]

    y = np.zeros(ncpgs)
    x = np.zeros(ncpgs)


    duration = 120
    while time.time() - start < duration:
        
        for i in range(ncpgs):
            y[i],x[i] = cpgUnits[i].get_motor_commands(start)      
            #y[i],x[i] = cpgUnits[i].euler(time.time() - cpgUnits[i].prev,cpgUnits[i].x,cpgUnits[i].y)
        

        cpg1.append(y[0])
        cpg2.append(y[1])
        cpg3.append(y[2])
        cpg4.append(y[3])
        cpg5.append(y[4])
        cpg6.append(y[5])

        cpg1x.append(x[0])
        cpg2x.append(x[1])
        t.append(time.time() - start)

    plt.figure()
    plt.plot(t,cpg1,'b', label = "cpg1")
    plt.plot(t,cpg2,'r',label = "cpg2")
    plt.plot(t,cpg3, 'b--',label = "cpg3")
    plt.plot(t,cpg4, 'r--',label = "cpg4")
    plt.plot(t,cpg5, 'b:',label = "cpg5")
    plt.plot(t,cpg6, 'r:',label = "cpg6")
    plt.xlabel("Time/s")
    plt.xlim(left=90)
    plt.legend()
    
    # plt.figure()
    # plt.plot(t,cpg1x,label = "x")
    # plt.plot(t,cpg1,label = "y")
    # plt.legend()
    # plt.figure()
    # plt.plot(t,cpgUnits[1].omegaList)
        
    plt.show()
    