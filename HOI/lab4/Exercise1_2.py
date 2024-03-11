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
    # Orientation2D("End-effector orientation", np.array([[np.pi]])),
    # Position2D("End-effector position", np.array([1.0, 0.5]).reshape(2, 1)),
    Configuration2D("End-effector config", np.array([1.0, 0.5, np.pi]).reshape(3, 1)),
    # JointPosition("Joint position", np.array([[], np.pi])),
]

# Simulation params
dt = 1.0 / 60.0

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
    line.set_data([], [])
    path.set_data([], [])
    point.set_data([], [])
    temp = np.vstack((np.random.uniform(-1, 1, size=(2, 1)), np.array([np.pi])))
    tasks[0].setDesired(temp)
    return line, path, point


# Simulation loop
def simulate(t):
    global tasks
    global robot
    global PPx, PPy

    ### Recursive Task-Priority algorithm
    # Initialize null-space projector
    P = np.eye(robot.getDOF())
    dq = np.zeros((robot.getDOF(), 1))
    # Initialize output vector (joint velocity)
    # Loop over tasks
    for task in tasks:
        task.update(robot)  # Update task state
        J = task.getJacobian()
        Jbar = J @ P  # Compute augmented Jacobian
        print((J).shape)
        dqi = DLS(Jbar, 0.1) @ (task.getError() - (J @ dq))
        dq += dqi  # Accumulate velocity
        P = P - np.linalg.pinv(Jbar) @ Jbar  # Update null-space projector
    ###

    # Update robot
    robot.update(dq, dt)

    # Update drawing
    PP = robot.drawing()
    line.set_data(PP[0, :], PP[1, :])
    PPx.append(PP[0, -1])
    PPy.append(PP[1, -1])
    path.set_data(PPx, PPy)
    point.set_data(tasks[0].getDesired()[0], tasks[0].getDesired()[1])

    return line, path, point

    # Update Poltting Data
    err1_plot.append(np.linalg.norm(err1))
    err2_plot.append(np.linalg.norm(err2))
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
    plt.plot(timestamp, err1_plot, label="e1 (end-effector position)")
    plt.plot(timestamp, err2_plot, label="e2 (joint 1 position)")

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
