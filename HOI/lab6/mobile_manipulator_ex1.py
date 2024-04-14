from lab6_robotics import * # Includes numpy import
import matplotlib.pyplot as plt
import matplotlib.patches as patch
import matplotlib.animation as anim
import matplotlib.transforms as trans

# Robot model
d = np.zeros(3)  # displacement along Z-axis
theta = np.array([0, np.pi / 4, np.pi / 4]) # rotation around Z-axis (theta)
alpha = np.zeros(3)  # displacement along X-axis
a = np.array([0.75, 0.5, 0.5])  # rotation around X-axis
revolute = [True, True, True]  # flags specifying the type of joints
robot = MobileManipulator(d, theta, a, alpha, revolute)  # Manipulator object
vel_threshold = 1.5
W = np.eye(robot.getDOF())
timestamp = []

# Joint limits definition
joint1_min = -0.5
joint1_max = 0.5
# Task definition


tasks = [ 
          JointLimits("Joint1-limits",np.zeros(1), 2, np.array([joint1_min , joint1_max])),
          Position2D("End-effector position", np.array([1.0, 0.5]).reshape(2,1), 5),
        ] 

# Simulation params
dt = 1.0/60.0

# Drawing preparation
fig = plt.figure()
ax = fig.add_subplot(111, autoscale_on=False, xlim=(-2, 2), ylim=(-2,2))
ax.set_title('Simulation')
ax.set_aspect('equal')
ax.grid()
ax.set_xlabel('x[m]')
ax.set_ylabel('y[m]')
rectangle = patch.Rectangle((-0.25, -0.15), 0.5, 0.3, color='blue', alpha=0.3)
veh = ax.add_patch(rectangle)
line, = ax.plot([], [], 'o-', lw=2) # Robot structure
path, = ax.plot([], [], 'c-', lw=1) # End-effector path
point, = ax.plot([], [], 'rx') # Target
PPx = []
PPy = []

# Simulation initialization
def init():
    global tasks, last_time
    line.set_data([], [])
    path.set_data([], [])
    point.set_data([], [])
    temp = np.random.uniform(-2, 2, size=(2, 1))
    tasks[-1].setDesired(temp)
    last_time = timestamp[-1] if timestamp else 0
    return line, path, point

# Simulation loop
def simulate(t):
    global tasks
    global robot
    global PPx, PPy
    global W
    
    ### Recursive Task-Priority algorithm
    P = np.eye(robot.getDOF())
    dq = np.zeros((robot.getDOF(), 1))
    dq_list = []
    
    # Initialize output vector (joint velocity)
    # Loop over tasks
    for task in tasks:
        task.update(robot)  # Update task state
        if task.active:
            J = task.getJacobian() # get task Jacobian
            Jbar = J @ P  # Compute augmented Jacobian
            
            
            dqi = DLS(Jbar, 0.1,W) @ ((task.getKmatrix() @ task.getError()) + task.getFeedForwardVel() - (J @ dq)) # control equation
            dq += dqi  # Accumulate velocity
            print("dq :",dq)

            P = P - np.linalg.pinv(Jbar) @ Jbar  # Update null-space projector
            dq_list.append(np.linalg.norm(dqi))
    ###
        
    # Scale velocity
    max_vel = max(dq_list)
    if max_vel > vel_threshold:
        dq = (dq / max_vel) * vel_threshold

    # Update robot
    robot.update(dq, dt)
    
    # Update drawing
    # -- Manipulator links
    PP = robot.drawing()
    line.set_data(PP[0,:], PP[1,:])
    PPx.append(PP[0,-1])
    PPy.append(PP[1,-1])
    path.set_data(PPx, PPy)
    point.set_data(tasks[-1].getDesired()[0], tasks[-1].getDesired()[1])
    # -- Mobile base
    eta = robot.getBasePose()
    veh.set_transform(trans.Affine2D().rotate(eta[2,0]) + trans.Affine2D().translate(eta[0,0], eta[1,0]) + ax.transData)

    timestamp.append(t + last_time)

    return line, veh, path, point

def plot_summary_joint():
    # Evolution of joint positions Plotting
    fig_joint = plt.figure()
    ax = fig_joint.add_subplot(111, autoscale_on=False, xlim=(0, 60), ylim=(-0.7, 2.6))
    ax.set_title("Error Plot")
    ax.set_xlabel("Time[s]")
    ax.set_ylabel("Error")
    ax.grid()
    qmin_plot = [tasks[0].qmin for _ in range(len(timestamp))]
    qmax_plot = [tasks[0].qmax for _ in range(len(timestamp))]

    plt.plot(timestamp, qmin_plot, linestyle='--') # joint limits
    plt.plot(timestamp, qmax_plot, linestyle='--') # joint limits
    print(tasks[0].err_plot)
    plt.plot(timestamp, tasks[0].err_plot, label="q1 (joint 1 position)")
    plt.plot(timestamp, tasks[-1].err_plot, label="e1 (end-effector position error)")
    ax.legend()

    plt.show()


# Run simulation
animation = anim.FuncAnimation(fig, simulate, np.arange(0, 10, dt), 
                                interval=10, blit=True, init_func=init, repeat=True)
plt.show()
plot_summary_joint()