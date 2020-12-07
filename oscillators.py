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

def get_motor_commands():
    a, b = 0, 100

    t = np.linspace(a, b, 2000)#

    #solve differential equation 
    sol = solve_ivp(hopf, [a, b], [1, 0], t_eval=t)

    #transofrm the result into angles
    
    hip = 512*sol.y[1] + 2048
    knee = []
    for i,val in enumerate(sol.y[1]):
        #print(i)
        if i < 1999:
            if val < sol.y[1][i+1]:
                knee.append(314*sol.y[0][i] + 2048)
            else:
                knee.append(2048)
        else:
            knee.append(knee[-1])

    return hip,knee,t,sol



if __name__ == '__main__':
    hip,knee,t,sol = get_motor_commands()

    # fig, axs = plt.subplots(2)
    # axs[0].plot(t, hip)
    # axs[1].plot(t, knee)
    
    fig, axs = plt.subplots(1)
    axs.plot(t, hip)
    axs.plot(t, knee, 'r')
    fig2, axs2 = plt.subplots(2)
    axs2[0].plot(t, sol.y[0])
    axs2[1].plot(t, sol.y[1])
    plt.show()