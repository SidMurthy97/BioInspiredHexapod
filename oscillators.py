import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt

#Model for the VDp oscillator, z is the initial conditino to begin with and then the outputs of the solver 
def vdp(t, z):
    epsilon = 0.1
    omega_sq1 = 1
    scaling1 = 1
    scaling2 = 1
    omega_sq2 = 1
    
    x1, y1,x2,y2 = z

    #return is then the new conditions 
    return [y1, epsilon*(1- scaling1*(x1**2))*y1 - omega_sq1*x1, y2,epsilon*(1- scaling2*(x2**2))*y2 - omega_sq2*x2 + 0.3*x1*y1]

def hopf(t,z):
    a,mu,omega = 1,1,1
    x,y = z

    #returns [x,y]
    return [a*(mu - x**2 - y**2)*x - omega*y, a*(mu - x**2 - y **2)*y + omega*x]

def angle_to_position(angles):
    conversion_factor = 11.4 # 11.4 units for each degree 
    offset = 2048
    
    positions = angles*conversion_factor + offset

    return positions

def get_motor_commands():
    a, b = 0, 30

    t = np.linspace(a, b, 1000)#

    #solve differential equation 
    sol = solve_ivp(hopf, [a, b], [1, 0], t_eval=t)

    #transofrm the result into angles
    
    hip = 45*sol.y[1] #amplitude is 45 degrees
    knee = []

    for i,val in enumerate(sol.y[1]):
        #print(i)
        if i < len(sol.y[0]) -2:
            if val < sol.y[1][i+1]:
                knee.append(30*sol.y[0][i]) #amplitude is 30 degrees
            else:
                knee.append(0)
        else:
            knee.append(knee[-1])

    knee = np.array(knee)
    return hip,knee,t



if __name__ == '__main__':
    hip,knee,t = get_motor_commands()
    # fig, axs = plt.subplots(2)
    # axs[0].plot(t, hip)
    # axs[1].plot(t, knee)
    plt.figure()
    plt.plot(t,angle_to_position(hip),"b",label= "Hip position")
    plt.plot(t,angle_to_position(knee),"-r", label = "Knee position")
    plt.xlabel("time")
    plt.ylabel("position")
    plt.legend()

    plt.figure()
    plt.plot(t,hip,"b",label= "Hip position")
    plt.plot(t,knee,"-r", label = "Knee position")
    plt.title("Angle profile with time")
    plt.xlabel("time")
    plt.ylabel("Angles")
    plt.legend()


    plt.show()