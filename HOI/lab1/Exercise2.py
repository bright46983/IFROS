# Import necessary libraries
from lab2_robotics import * # Includes numpy import
import matplotlib.pyplot as plt
import matplotlib.animation as anim

# Robot definition
d_init = np.zeros(2)           # displacement along Z-axis
q_init = np.array([0.2, 0.5])  # rotation around Z-axis (theta)
a_init = np.array([0.75, 0.5]) # displacement along X-axis
alpha_init = np.zeros(2)       # rotation around X-axis 
d = d_init
q = q_init
a = a_init
alpha = alpha_init
revolute = [True, True]
sigma_d = np.array([0.3, 0.8])
K = np.diag([1, 1])
current_controller = "transpose"



# Simulation params
dt = 1.0/60.0

# Drawing preparation
fig = plt.figure()
ax = fig.add_subplot(111, autoscale_on=False, xlim=(-2, 2), ylim=(-2,2))
ax.set_title('Simulation')
ax.set_aspect('equal')
ax.grid()
line, = ax.plot([], [], 'o-', lw=2) # Robot structure
path, = ax.plot([], [], 'c-', lw=1) # End-effector path
point, = ax.plot([], [], 'rx') # Target
PPx = []
PPy = []

# Error Plotting
norm_err = {"transpose":[], "inverse":[],"DLS":[]}
timestamp = []

# Simulation initialization
def init():
    line.set_data([], [])
    path.set_data([], [])
    point.set_data([], [])
    return line, path, point

# Simulation loop
def simulate(t):
    global d, q, a, alpha, revolute, sigma_d
    global PPx, PPy

    # Update robot
    T = kinematics(d, q, a, alpha)
    J = jacobian(T, revolute)
    P = robotPoints2D(T)

    # Update control
    sigma = np.array([P[0,-1], P[1,-1]])    # Position of the end-effector
    err =   sigma_d - sigma   # Control error (position error)
    
    # Control solutions
    dq = controller(current_controller, J)[:,0:2] @ (K @ err)    
    q += dt * dq

    # Update drawing
    line.set_data(P[0,:], P[1,:])
    PPx.append(P[0,-1])
    PPy.append(P[1,-1])
    path.set_data(PPx, PPy)
    point.set_data(sigma_d[0], sigma_d[1])

    return line, path, point

def controller(type, J):
    if type == "transpose":
        return J.T
    elif type == "inverse":
        return np.linalg.pinv(J)
    elif type == "DLS":
        return DLS(J,0.1)



   

def plot_summary():
    # Evolution of joint positions Plotting
    fig_joint = plt.figure()
    ax = fig.add_subplot(222, autoscale_on=True)
    ax.set_title('joint positions')
    ax.set_xlabel('Time[s]')
    ax.set_ylabel('Error[m]')
    ax.set_aspect('equal')
    ax.grid()
    plt.plot(timestamp,norm_err["transpose"],label='transpose')
    plt.plot(timestamp,norm_err["inverse"],label='psudoinverse')
    plt.plot(timestamp,norm_err["DLS"],label='DLS')
    plt.show()

# Run simulation
current_controller = "transpose"
animation = anim.FuncAnimation(fig, simulate, np.arange(0, 10, dt), 
                                interval=10, blit=True, init_func=init, repeat=False)
plt.show()



plot_summary()