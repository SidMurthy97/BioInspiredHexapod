import matplotlib.pyplot as plt
import numpy as np
import math 
import time 
from oscillators import angle_to_position
import pprint


class CPG():

    def __init__(self,prev):
        #hopf parameters
        self.hopfMu = 1
        self.stanceF = (1/3)*math.pi
        self.swingF = math.pi
        
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
        a,mu= 1,1

        #angular frequency can be modulated using current position 
        omega = self.stanceF/(math.exp(-10*x) + 1) + self.swingF/(math.exp(10*x) + 1)


        dx = a*(mu - x**2 - y**2)*x - omega*y
        dy =  a*(mu - x**2 - y **2)*y + omega*x
        #print(self.coupledCPG)
        if self.coupledCPG[0] != None:
            
            # deltaSum = 0
            # #print(self.coupledCPG)
            # for couple in self.coupledCPG:
            #     phase = couple[1]
            #     unit = couple[0]
            #     deltaSum +=  (unit.y * np.cos(phase) - unit.x * np.sin(phase))

            # return dx, dy + deltaSum

            
            deltaSumX,deltaSumY = 0,0
            #print(self.coupledCPG)
            for couple in self.coupledCPG:
                #print(couple)
                phase = couple[1]
                unit = couple[0]

                deltaSumX = (unit.x + unit.y)/(math.sqrt(unit.x**2 + unit.y**2)) * np.sin(phase)
                deltaSumY = (unit.x + unit.y)/(math.sqrt(unit.x**2 + unit.y**2)) * np.cos(phase)
                #print(deltaSumX,deltaSumY)
            return dx - deltaSumX, dy + deltaSumY
            
        else:
            return dx,dy
    
    def couple(self,influencerUnit,phase):
        self.coupledCPG.append([influencerUnit,phase])


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
            hip = math.pi/12 *y
            knee = math.pi/6*x if x > 0 else 0 

            return hip,knee



if __name__ == "__main__":
    xr,yr,t,omegaList = [],[], [],[]
    
    start = time.time()

    cpgUnits = []
    ncpgs = 6

    #lists to store cpg outputs
    cpg1,cpg2,cpg3,cpg4,cpg5,cpg6 = [],[],[],[],[],[]
    
    #initialise CPGs
    for i in range(ncpgs):
            cpgUnits.append(CPG(start))
    
    #NEED BETTER WAY TO DEFINE CPGS AND PHASE RELATIONSHIPS BETWEEN LEGS 
    #couple CPGs
    phase = math.pi
    for i in range(ncpgs):
        prevUnit = cpgUnits[(i-1)%ncpgs]
        nextUnit = cpgUnits[(i+1)%ncpgs]
        
        cpgUnits[i].coupledCPG = [[prevUnit,phase],[nextUnit,phase]]

    # cpgUnits[-1].coupledCPG = [[cpgUnits[0],-phase]]

    # for i in range(ncpgs):
    #     prevUnit = cpgUnits[(i-1)%ncpgs]
    #     nextUnit = cpgUnits[(i+1)%ncpgs]

    #     cpgUnits[i].coupledCPG = [[nextUnit,math.pi/3],[prevUnit,-math.pi/3]]


    y = np.zeros(ncpgs)

    while time.time() - start < 30:
        
        for i in range(ncpgs):
            y[i],_ = cpgUnits[i].euler(time.time() - cpgUnits[i].prev,cpgUnits[i].x,cpgUnits[i].y)
        

        cpg1.append(y[0])
        cpg2.append(y[1])
        cpg3.append(y[2])
        cpg4.append(y[3])
        cpg5.append(y[4])
        cpg6.append(y[5])
        t.append(time.time() - start)

    plt.figure()
    plt.plot(t,cpg1, label = "cpg1")
    plt.plot(t,cpg2, label = "cpg2")
    plt.plot(t,cpg3, '--',label = "cpg3")
    plt.plot(t,cpg4, '--',label = "cpg4")
    plt.plot(t,cpg5, '--',label = "cpg5")
    plt.plot(t,cpg6, '--',label = "cpg6")

    plt.xlabel("Time/s")
    # plt.ylabel("Angles")
    # plt.title("Expected Angle Profile") 
    plt.legend()
    

    # for i,x in enumerate(xr):
    #     print(i,x)
    #     test.append(2*math.pi/(math.exp(-10*x) + 1) + math.pi/(math.exp(10*x) + 1))
        
    plt.show()
    