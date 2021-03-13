import matplotlib.pyplot as plt
import numpy as np
import math 
import time 
from oscillators import angle_to_position



class CPG():

    def __init__(self,prev,coupledCPG):
        self.hopfMu = 1
        self.hopfOmega = math.pi
        self.torqueFeedback = 0
        self.attenuation = 1/30
        self.offset = 0
        self.n = 0
        self.k = 60
        self.elevator = False
        self.criticalHipPos = 0
        self.criticalKneePos = 0
        self.release = False

        self.prev = prev
        self.x = 1
        self.y = 0

        self.coupledCPG = coupledCPG
        self.delta = 1
        self.phase = math.pi
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
        omega = (math.pi)/(math.exp(-10*x) + 1) + math.pi/(math.exp(10*x) + 1)

        if self.coupledCPG:
            return [a*(mu - x**2 - y**2)*x - omega*y, a*(mu - x**2 - y **2)*y + omega*x + self.coupledCPG.y * math.cos(self.phase) - self.coupledCPG.x * math.sin(self.phase)]
        else:
            return [a*(mu - x**2 - y**2)*x - omega*y, a*(mu - x**2 - y **2)*y + omega*x]
    def euler(self,t,x,y):
        
        #set new timestamp to calculate next step size
        self.prev = time.time()

        #get gradients using the hopf oscillator equations
        dx,dy = self.hopf([x,y])

        #estimate next value using euler method 
        x = x + t*dx
        y = y + t*dy

        #return new coordinates
        return x,y

    def vdp(self,t,z):
        b,w = 1,math.pi
        x,y = z

        x = (2/math.sqrt(b))*np.cos(w*t) 
        y = ((2*w)/math.sqrt(b))*np.sin(w*t) 

        return x,y

    def get_motor_commands(self,start,realWorld = False):

        #use euler method to solve hopf equations     
        self.x,self.y = self.euler(time.time() - self.prev,self.x,self.y)
        
        #convert oscillator outputs to motor positions 
        x,y = self.x,self.y
        
        if realWorld: #if running on hardware convert to position
            hip = 30 *y
            knee = 30*x if x > 0 else 0 

            return angle_to_position(hip),angle_to_position(knee)
        
        else: #if running in simulation, give radian angles
            hip = math.pi /6 *y
            knee = math.pi/6*x if x > 0 else 0 

            return hip,knee



if __name__ == "__main__":
    xr,yr,t,omegaList = [],[], [],[]
    
    start = time.time()
    cpg = CPG(start,None)
    while time.time() - start < 5:
        
        hip,knee = cpg.get_motor_commands(start,True) #get hip and knee commands by solving Hopf equations

        xr.append((hip))
        yr.append((knee))
        t.append(time.time() - start)

    plt.figure()
    plt.plot(t,xr,"b", label = "Hip")
    plt.plot(t,yr,"r", label = "Knee")

    plt.xlabel("Time/s")
    plt.ylabel("Angles")
    plt.title("Expected Angle Profile") 
    plt.legend()
    

    # for i,x in enumerate(xr):
    #     print(i,x)
    #     test.append(2*math.pi/(math.exp(-10*x) + 1) + math.pi/(math.exp(10*x) + 1))
        
    plt.show()
    