# Import necessary libraries
from lab2_robotics import * # Includes numpy import
import matplotlib.pyplot as plt
import matplotlib.animation as anim

# Robot definition
d = np.zeros(2)           # displacement along Z-axis
q = np.array([[0.2, 0.5],[0.2, 0.5],[0.2, 0.5]])  # rotation around Z-axis (theta)
a = np.array([0.75, 0.5]) # displacement along X-axis
alpha = np.zeros(2)       # rotation around X-axis 
revolute = [True, True]
sigma_d = np.array([1, 1])
K = np.diag([1, 1]) 
controller_list =  ["transpose","inverse","DLS"] 

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
norm_err = [[],[],[]]
timestamp = []

# Simulation initialization
def init():
    line.set_data([], [])
    path.set_data([], [])
    point.set_data([], [])
    return line, path, point

# Simulation loop
def simulate(t):
    global d, q, a, alpha, revolute, sigma_d, norm_err
    global PPx, PPy

    # Update robot state for each controller type
    T = [kinematics(d, q[i], a, alpha) for i in range(len(controller_list))] 
    J = [jacobian(T[i], revolute) for i in range(len(controller_list))]
    P = [robotPoints2D(T[i]) for i in range(len(controller_list))]

    # Update control
    sigma = [np.array([P[i][0,-1], P[i][1,-1]]) for i in range(len(controller_list))]    # Position of the end-effector for each solution
    err =   [sigma_d - sigma[i] for i in range(len(controller_list))]   # Control error (position error)
    
    
    # Control solutions
    dq = [controller(controller_list[i], J[i][:2,:]) @ (K @ err[i]) for i in range(len(controller_list)) ]  # calculate the velocity command 
                                                                                                             # controller matrix needed slicing to match the end-effector dimensions 
    for i in range(len(controller_list)):
        q[i] += dt * dq[i]
        #Error Poltting Elements
        norm_err[i].append(np.linalg.norm(err[i]))  # Norm error for each controller
    print(controller(controller_list[0], J[0][:2,:]))
    timestamp.append(t)

    # Update drawing 
    P = robotPoints2D(T[1]) 
    line.set_data(P[0,:], P[1,:])
    PPx.append(P[0,-1])
    PPy.append(P[1,-1])
    path.set_data(PPx, PPy)
    point.set_data(sigma_d[0], sigma_d[1])

    return line, path, point

def controller(type, J):
    # return the controller matrix according to the controller type
    if type == "transpose":
        return J.T
    elif type == "inverse":
        return np.linalg.pinv(J)
    elif type == "DLS":
        return DLS(J,0.1)
    else:
        print("Invalid controller type!")
        return J.T



   

def plot_summary():
    # Evolution of joint positions Plotting
    fig_joint = plt.figure()
    ax = fig_joint.add_subplot(111, autoscale_on=False, xlim=(0, 20), ylim=(0,1.7))
    ax.set_title('joint positions')
    ax.set_xlabel('Time[s]')
    ax.set_ylabel('Error[m]')
    ax.grid()
    plt.plot(timestamp,norm_err[0],label='transpose')
    plt.plot(timestamp,norm_err[1],label='psudoinverse')
    plt.plot(timestamp,norm_err[2],label='DLS')

    ax.legend()

    plt.show()

# Run simulation
current_controller = "transpose"
animation = anim.FuncAnimation(fig, simulate, np.arange(0, 20, dt), 
                                interval=10, blit=True, init_func=init, repeat=False)
plt.show()



plot_summary()