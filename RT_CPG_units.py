import matplotlib.pyplot as plt
import numpy as np
import math 
import time 
from oscillators import angle_to_position



class CPG():

    def __init__(self):
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
    
    
    def hopf(self,t):
        
        #keep count of the number of times this function has been called to make online averaging accurate
        #averages only the last k values
        if self.n < self.k:
            self.n+=1
        else:
            self.n = self.k
        
        mu,omega = self.hopfMu,self.hopfOmega

        #Online averaging to make the effect of large torque last longer.
        self.offset = self.offset + (self.torqueFeedback * self.attenuation - self.offset)/self.n
        #  

        
        if self.torqueFeedback > 50 and t > 1 and self.release == False:
            self.elevator = True
            offset = 0
        else:
            self.release = False
            offset = self.offset
            
        
        
        #add offset only to knee
        x = math.sqrt(mu)*np.cos(omega*t) + offset
        y = math.sqrt(mu)*np.sin(omega*t)
        #phase shifted values 
        xd = math.sqrt(mu)*np.cos(omega*t + math.pi)
        yd = math.sqrt(mu)*np.sin(omega*t + math.pi)
        
    
        return x,y,xd,yd


    def vdp(self,t,z):
        b,w = 1,math.pi
        x,y = z

        x = (2/math.sqrt(b))*np.cos(w*t) 
        y = ((2*w)/math.sqrt(b))*np.sin(w*t) 

        return x,y

    def get_motor_commands(self,start,realWorld = False):
            
        x,y,xd,yd = self.hopf(time.time() - start)
        
        if realWorld:
            hip = 30 *y
            knee = 30*x if x > 0 else 0 

            hipd = 30 *yd
            kneed = 30*xd if xd > 0 else 0 
            return angle_to_position(hip),angle_to_position(knee)
        else:
            hip = math.pi /6 *y
            knee = math.pi/6*x if x > 0 else 0 

            hipd = math.pi /6 *yd
            kneed = math.pi/6*xd if xd > 0 else 0 
            return hip,knee,hipd,kneed



if __name__ == "__main__":
    initial_conditions = [0,0]
    x,y = initial_conditions
    xr,yr,t = [],[], []
    start = time.time()
    cpg = CPG()

    while time.time() - start < 10:
        
        hip,knee = cpg.get_motor_commands(start,True)
        #knee,hip,_,_ = cpg.hopf(time.time() - start)

        xr.append((hip))
        yr.append((knee))
        t.append(time.time() - start)

    plt.figure()
    plt.plot(t,xr,"b", label = "Hip")
    plt.plot(t,yr,"r", label = "Knee")

    # plt.plot(t,-1*xr)
    # plt.plot(t,-1*yr)
    plt.xlabel("Time/s")
    plt.ylabel("Angles")
    plt.title("Expected Angle Profile") 
    plt.legend()
    plt.show()
    