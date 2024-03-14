from lab4_robotics import *  # Includes numpy import
import matplotlib.pyplot as plt
import matplotlib.animation as anim

# Robot model - 3-link manipulator
d = np.zeros(3)  # displacement along Z-axis
theta = np.array([0, np.pi / 4, np.pi / 4]).reshape(
    3, 1
)  # rotation around Z-axis (theta)
alpha = np.zeros(3)  # displacement along X-axis
a = np.array([0.75, 0.5, 0.5])  # rotation around X-axis
revolute = [True, True, True]  # flags specifying the type of joints
robot = Manipulator(d, theta, a, alpha, revolute)  # Manipulator object

# Task hierarchy definition
tasks = [
    
    Position2D("End-effector position", np.array([1.0, 0.5]).reshape(2, 1), link=3),
    Orientation2D("End-effector orientation", np.array([[np.pi]]), link=2),
    #Position2D("End-effector position", np.array([1.0, 0.5]).reshape(2, 1), 1),
    # Configuration2D("End-effector config", np.array([1.0, 0.5, np.pi]).reshape(3, 1)),
    #JointPosition("Joint position", np.array([0]),0),
]

# Set K matrix for tash 1
K = 2
tasks[0].setKmatrix(K)
# Set up FFV
FFV = 0.0
tasks[0].setFeedForwardVel(FFV)

# Simulation params
dt = 1.0 / 60.0
timestamp = []
vel_threshold = 1.5
# Drawing preparation
fig = plt.figure()
ax = fig.add_subplot(111, autoscale_on=False, xlim=(-2, 2), ylim=(-2, 2))
ax.set_title("Simulation")
ax.set_aspect("equal")
ax.grid()
ax.set_xlabel("x[m]")
ax.set_ylabel("y[m]")
(line,) = ax.plot([], [], "o-", lw=2)  # Robot structure
(path,) = ax.plot([], [], "c-", lw=1)  # End-effector path
(point,) = ax.plot([], [], "rx")  # Target
PPx = []
PPy = []


# Simulation initialization
def init():
    global tasks
    global last_time
    line.set_data([], [])
    path.set_data([], [])
    point.set_data([], [])
    temp = np.random.uniform(-1, 1, size=(2, 1))
    tasks[0].setDesired(temp)
    last_time = timestamp[-1] if timestamp else 0
    return line, path, point


# Simulation loop
def simulate(t):
    global tasks
    global robot
    global PPx, PPy, last_time

    ### Recursive Task-Priority algorithm
    # Initialize null-space projector
    P = np.eye(robot.getDOF())
    dq = np.zeros((robot.getDOF(), 1))
    dq_list = []
    
    # Initialize output vector (joint velocity)
    # Loop over tasks
    for task in tasks:
        task.update(robot)  # Update task state
        J = task.getJacobian() # get task Jacobian
        Jbar = J @ P  # Compute augmented Jacobian
        dqi = DLS(Jbar, 0.1) @ ((task.getKmatrix() @ task.getError()) + task.getFeedForwardVel() - (J @ dq)) # control equation
        dq += dqi  # Accumulate velocity
        P = P - np.linalg.pinv(Jbar) @ Jbar  # Update null-space projector

        dq_list.append(np.linalg.norm(dqi)) # control memory for velcoity scaling
    ###
        
    # Scale velocity
    max_vel = max(dq_list)
    if max_vel > vel_threshold:
        dq = (dq / max_vel) * vel_threshold

    # Update robot
    robot.update(dq, dt)

    # Update drawing
    PP = robot.drawing()
    line.set_data(PP[0, :], PP[1, :])
    PPx.append(PP[0, -1])
    PPy.append(PP[1, -1])
    path.set_data(PPx, PPy)
    point.set_data(tasks[0].getDesired()[0], tasks[0].getDesired()[1])
    timestamp.append(t + last_time)

    return line, path, point

    


def plot_summary():
    # Evolution of joint positions Plotting
    fig_joint = plt.figure()
    ax = fig_joint.add_subplot(111, autoscale_on=False, xlim=(0, 60), ylim=(-1, 2))
    ax.set_title("Task-Priority (two tasks)")
    ax.set_xlabel("Time[s]")
    ax.set_ylabel("Error")
    ax.grid()
    plt.plot(timestamp, tasks[0].err_plot, label="e1 ({})".format(tasks[0].name))
    plt.plot(timestamp, tasks[1].err_plot, label="e2 ({})".format(tasks[1].name))

    ax.legend()

    plt.show()


# Run simulation
animation = anim.FuncAnimation(
    fig,
    simulate,
    np.arange(0, 10, dt),
    interval=10,
    blit=True,
    init_func=init,
    repeat=True,
)
plt.show()
plot_summary()
