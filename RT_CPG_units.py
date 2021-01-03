import matplotlib.pyplot as plt
import numpy as np
import math 
import time 
from oscillators import angle_to_position


def hopf(t):
    a,mu,omega = 1,1,math.pi

    x = math.sqrt(mu)*np.cos(omega*t)
    y = math.sqrt(mu)*np.sin(omega*t)
    return x,y


def vdp(t,z):
    b,w = 1,math.pi
    x,y = z

    x = (2/math.sqrt(b))*np.cos(w*t) 
    y = ((2*w)/math.sqrt(b))*np.sin(w*t) 

    return x,y

def get_motor_commands(start):
        
    x,y = hopf(time.time() - start)
    hip = 45*y
    knee = 30*x if x > 0 else 0 

    return angle_to_position(hip),angle_to_position(knee)




if __name__ == "__main__":
    initial_conditions = [0,0]
    x,y = initial_conditions
    xr,yr,t = [],[], []
    start = time.time()
    
    while time.time() - start < 10:
        
        x,y = hopf(time.time() - start)
        hip = 45*y
        knee = 30*x if x > 0 else 0 

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
    plt.show()
    