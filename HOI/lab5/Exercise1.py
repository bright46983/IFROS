from lab5_robotics import * # Includes numpy import
import matplotlib.pyplot as plt
import matplotlib.animation as anim
import matplotlib.patches as patch

# Robot model - 3-link manipulator
d = np.zeros(3)  # displacement along Z-axis
theta = np.array([0, np.pi / 4, np.pi / 4]).reshape(
    3, 1
)  # rotation around Z-axis (theta)
alpha = np.zeros(3)  # displacement along X-axis
a = np.array([0.75, 0.5, 0.5])  # rotation around X-axis
revolute = [True, True, True]  # flags specifying the type of joints
robot = Manipulator(d, theta, a, alpha, revolute)  # Manipulator object
vel_threshold = 1.5

# Obstacle definition
obstacle_pos = np.array([0.0, 1.0]).reshape(2,1)
obstacle_r = 0.5
obstacle_pos2 = np.array([-0.5, -0.75]).reshape(2,1)
obstacle_r2 = 0.4
obstacle_pos3 = np.array([0.75, -0.5]).reshape(2,1)
obstacle_r3 = 0.275
obstacles_list = [obstacle_pos, obstacle_pos2, obstacle_pos3]
obstacles_r_list = [obstacle_r, obstacle_r2, obstacle_r3]

# Joint limits definition
joint1_min = -0.5
joint1_max = 0.5
# ee_min = -0.5
# ee_max = 0.5

# Task hierarchy definition
tasks = [ 
        #   JointLimits("Joint1-limits",np.zeros(1), 0, np.array([joint1_min , joint1_max])),

          Obstacle2D("Obstacle avoidance", obstacle_pos, np.array([obstacle_r, obstacle_r+0.025])),
          Obstacle2D("Obstacle avoidance", obstacle_pos2, np.array([obstacle_r2, obstacle_r2+0.025])),
          Obstacle2D("Obstacle avoidance", obstacle_pos3, np.array([obstacle_r3, obstacle_r3+0.025])),
           Position2D("End-effector position", np.array([-1, 1]).reshape(2,1))
        ] 

# Simulation params
dt = 1.0/60.0
timestamp = []


# Drawing preparation
fig = plt.figure()
ax = fig.add_subplot(111, autoscale_on=False, xlim=(-2, 2), ylim=(-2,2))
ax.set_title('Simulation')
ax.set_aspect('equal')
ax.grid()
ax.set_xlabel('x[m]')
ax.set_ylabel('y[m]')
color = ['red', 'green', 'blue']
for i in range(len(obstacles_list)):
    ax.add_patch(patch.Circle(obstacles_list[i].flatten(), obstacles_r_list[i], color=color[i], alpha=0.3))
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
    last_time = timestamp[-1] if timestamp else 0
    temp = np.random.uniform(-1.25, 1.25, size=(2, 1))
    tasks[-1].setDesired(temp)
    return line, path, point

# Simulation loop
def simulate(t):
    global tasks
    global robot
    global PPx, PPy, last_time
    
    ### Recursive Task-Priority algorithm (w/set-based tasks)
    # The algorithm works in the same way as in Lab4. 
    # The only difference is that it checks if a task is active.
    ###
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
            print("DLS:",DLS(Jbar, 0.1).shape)
            print("J :",J)
            print("dq :",dq)

            dqi = DLS(Jbar, 0.1) @ ((task.getKmatrix() @ task.getError()) + task.getFeedForwardVel() - (J @ dq)) # control equation
            dq += dqi  # Accumulate velocity
            P = P - np.linalg.pinv(Jbar) @ Jbar  # Update null-space projector
            dq_list.append(np.linalg.norm(dqi))
    ###
        
    # Scale velocity
    max_vel = max(dq_list)
    if max_vel > vel_threshold:
        dq = (dq / max_vel) * vel_threshold

    # Update robot
    robot.update(dq, dt)
    #
    
    # Update drawing
    PP = robot.drawing()
    line.set_data(PP[0,:], PP[1,:])
    PPx.append(PP[0,-1])
    PPy.append(PP[1,-1])
    path.set_data(PPx, PPy)
    point.set_data(tasks[-1].getDesired()[0], tasks[-1].getDesired()[1])

    timestamp.append(t + last_time)
    
    return line, path, point

def plot_summary():
    # Evolution of joint positions Plotting
    fig_joint = plt.figure()
    ax = fig_joint.add_subplot(111, autoscale_on=False, xlim=(0, 60), ylim=(-0.2, 2.6))
    ax.set_title("Task-Priority (two tasks)")
    ax.set_xlabel("Time[s]")
    ax.set_ylabel("Error")
    ax.grid()
    plt.plot(timestamp, tasks[0].err_plot, label="d1 (distance to obstacle)")
    plt.plot(timestamp, tasks[1].err_plot, label="d2 (distance to obstacle)")
    plt.plot(timestamp, tasks[2].err_plot, label="d3 (distance to obstacle)")
    plt.plot(timestamp, tasks[-1].err_plot, label="e1 (end-effector position error)")


    ax.legend()

    plt.show()

def plot_summary_joint():
    # Evolution of joint positions Plotting
    fig_joint = plt.figure()
    ax = fig_joint.add_subplot(111, autoscale_on=False, xlim=(0, 60), ylim=(-0.7, 2.6))
    ax.set_title("Task-Priority (two tasks)")
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
plot_summary()
