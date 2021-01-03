import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt

#Model for the VDp oscillator, z is the initial conditino to begin with and then the outputs of the solver 
def vdp(t, z):
    epsilon = 0.1
    omega = 1
    b = 2
    
    x1, y1 = z

    #return is then the new conditions 
    return [y1, epsilon*(1- b*(x1**2))*y1 - omega*x1]

def hopf(t,z):
    a,mu,omega = 1,2,1
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

    t = np.linspace(a, b, 1000)

    #solve differential equation 
    #sol = solve_ivp(hopf, [a, b], [1, 0], t_eval=t)
    sol = solve_ivp(vdp, [a, b], [1, 0], t_eval=t)

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
    return hip,knee,t,sol
    

def get_vector_portrait():
    l = 2.0

    # no. of points along x/y axis
    n = 101

    # X and Y dimensions
    x1 = np.linspace(-l, l, n)
    x2 = np.linspace(-l, l, n)

    X1, X2 = np.meshgrid(x1, x2) #there are the points at which the vector is evaluated

    U, V = vdp(0,[X1, X2])

    # Resultant velocity
    vels = np.hypot(U, V)

    # Slice interval for quiver
    slice_interval = 6

    # Slicer index for smoother quiver plot
    # General note: Adjust the slice interval and scale accordingly to get the required arrow size.
    # Also, the units, and angles units are also responsible.
    skip = (slice(None, None, slice_interval), slice(None, None, slice_interval))

    return X1[skip],X2[skip], U[skip],V[skip], vels[skip]



if __name__ == '__main__':
    hip,knee,t,sol = get_motor_commands()
    vector_x,vector_y,U,V,velocity = get_vector_portrait()

    # plt.figure()
    # plt.plot(sol.y[0],sol.y[1])
    # plt.title("Limit Cycle of VDP oscillator")
    # plt.xlabel("x")
    # plt.ylabel("y")

    plt.figure()
    plt.plot(t,sol.y[1],"b",label= "y")
    plt.plot(t,sol.y[0],"-r", label = "x")
    plt.xlabel("time")
    plt.ylabel("position")
    plt.title("Hopf oscillator outputs with b = 1, $\omega$ = 1")
    plt.legend()


    plt.figure()
    plt.plot(sol.y[0],sol.y[1],'-r')
    Quiver = plt.quiver(vector_x,vector_y,
                        U, V,
                        velocity,
                        units='height',
                        angles='uv',
                        scale=50,
                        pivot='mid',
                        # color='blue',
                        cmap=plt.cm.seismic
                        )
    plt.title("Hopf Oscillator Vector Portrait with $\epsilon = 0.1$")
    plt.xlabel("x")
    plt.ylabel("y")
    plt.colorbar(Quiver)
    plt.xticks()
    plt.yticks()
    #plt.axis([-l, l, -l, l])
    plt.grid()


    # plt.figure()
    # plt.plot(t,angle_to_position(hip),"b",label= "Hip position")
    # plt.plot(t,angle_to_position(knee),"-r", label = "Knee position")
    # plt.xlabel("time")
    # plt.ylabel("position")
    # plt.title("Angle profile with time")
    # plt.legend()

    plt.show()