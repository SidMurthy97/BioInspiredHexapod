import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt

#z is the initial condition
def vdp(t, z):
    x, y = z

    #return is then the new conditions 
    return [y, mu*(1 - x**2)*y - x]

a, b = 0, 10

mus = [1] #[0, 1, 2]
styles = ["-"] #["-", "--", ":"]
t = np.linspace(a, b, 1000)

for mu, style in zip(mus, styles):

    sol = solve_ivp(vdp, [a, b], [1, 0], t_eval=t)
    plt.plot(sol.y[0], sol.y[1], style)
  
# make a little extra horizontal room for legend
plt.xlim([-3,3])    
plt.legend([f"$\mu={m}$" for m in mus])
plt.show()