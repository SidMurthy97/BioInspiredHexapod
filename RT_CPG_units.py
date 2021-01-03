import matplotlib.pyplot as plt
import numpy as np
import math 
import time 

def hopf(t,z):
    a,mu,omega = 1,1,math.pi
    x,y = z

    x = math.sqrt(mu)*np.cos(omega*t)
    y = math.sqrt(mu)*np.sin(omega*t)
    return x,y


def vdp(t,z):
    b,w = 1,math.pi
    x,y = z

    x = (2/math.sqrt(b))*np.cos(w*t) 
    y = ((2*w)/math.sqrt(b))*np.sin(w*t) 

    return x,y

if __name__ == "__main__":
    initial_conditions = [0,0]
    x,y = initial_conditions
    xr,yr = [],[]
    start = time.time()
    
    while time.time() - start < 10:
        
        x,y = hopf(time.time() - start,[x,y])
        xr.append(x)
        yr.append(y)


plt.figure()
plt.plot(xr,yr)

plt.figure()
plt.plot(xr)
plt.plot(yr)
plt.show()
    