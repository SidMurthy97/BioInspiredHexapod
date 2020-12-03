import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt

#Model for the VDp oscillator, z is the initial conditino to begin with and then the outputs of the solver 
def vdp(t, z):
    epsilon = 0.1
    omega_sq1 = 1
    scaling1 = 1
    omega_sq2 = 1
    
    x1, y1,x2,y2 = z

    #return is then the new conditions 
    return [y1, epsilon*(1- scaling1*(x1**2))*y1 - omega_sq1*x1, y2,epsilon*(1- scaling2*(x2**2))*y2 - omega_sq2*x2 + 0.3*x1*y1]

def hopf(t,z):
    a,mu,omega = 1,1,1
    x,y = z

    return [a*(mu - x**2 - y**2)*x - omega*y, a*(mu - x**2 - y **2)*y + omega*x]

a, b = 0, 100

t = np.linspace(a, b, 10000)#

#solve differential equation 
sol = solve_ivp(hopf, [a, b], [1, 0], t_eval=t)

plt.figure()
plt.plot(sol.y[0], sol.y[1])
plt.title("Limit Cycle")

fig, axs = plt.subplots(2)
axs[0].plot(t, sol.y[0])
axs[1].plot(t, sol.y[1])
plt.show()