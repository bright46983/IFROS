# Import necessary libraries
from lab2_robotics import *  # Includes numpy import
import matplotlib.pyplot as plt
import matplotlib.animation as anim
import numpy as np


# Robot definition (3 revolute joint planar manipulator)
d = np.zeros(3)  # displacement along Z-axis
q = np.array([0, np.pi / 4, np.pi / 4])  # rotation around Z-axis (theta)
alpha = np.zeros(3)  # displacement along X-axis
a = np.array([0.75, 0.5, 0.5])  # rotation around X-axis
revolute = [True, True, True]  # flags specifying the type of joints
K = np.diag([1, 1])

# Setting desired position of end-effector to the current one
T = kinematics(
    d, q.flatten(), a, alpha
)  # flatten() needed if q defined as column vector !
sigma_d = T[-1][0:2, 3].reshape(2, 1)

# Simulation params
dt = 1.0 / 60.0
Tt = 10  # Total simulation time
tt = np.arange(0, Tt, dt)  # Simulation time vector

# Drawing preparation
fig = plt.figure()
ax = fig.add_subplot(111, autoscale_on=False, xlim=(-2, 2), ylim=(-2, 2))
ax.set_title("Simulation")
ax.set_aspect("equal")
ax.set_xlabel("x[m]")
ax.set_ylabel("y[m]")
ax.grid()
line, = ax.plot([], [], "o-", lw=2)  # Robot structure
path, = ax.plot([], [], "c-", lw=1)  # End-effector path
point, = ax.plot([], [], "rx")  # Target
PPx = []
PPy = []
q1 = []
q2 = []
q3 = []
timestamp = []

# Simulation initialization
def init():
    line.set_data([], [])
    path.set_data([], [])
    point.set_data([], [])
    return line, path, point


# Simulation loop
def simulate(t):
    global q, a, d, alpha, revolute, sigma_d
    # Update robot
    T = kinematics(d, q.flatten(), a, alpha)
    J = jacobian(T, revolute)

    # Update control
    sigma = T[-1][0:2, -1].reshape(2, 1)  # Current position of the end-effector
    err = sigma_d - sigma  # Error in position

    Jbar = J[:2, :]  # Task Jacobian
    Jinv = np.linalg.pinv(J[:2, :])
    P = np.eye(3) - (Jinv @ Jbar)  # Null space projector
    y = np.array([np.cos(t), 1.5 * np.sin(t), -1.75 * np.sin(t)]).reshape(3, 1)  # Arbitrary joint velocity
    dq = (Jinv @ (K @ err)) + (P @ y)  # Control signal
    q = q + dt * dq.reshape(3)  # Simulation update

    # Update drawing
    PP = robotPoints2D(T)
    line.set_data(PP[0, :], PP[1, :])
    PPx.append(PP[0, -1])
    PPy.append(PP[1, -1])
    path.set_data(PPx, PPy)
    point.set_data(sigma_d[0], sigma_d[1])

    # Update memory for plotting
    q1.append(q[0])
    q2.append(q[1])
    q3.append(q[2])
    timestamp.append(t)

    return line, path, point


def plot_summary():
    # Evolution of joint positions Plotting
    fig_joint = plt.figure()
    ax = fig_joint.add_subplot(111, autoscale_on=False, xlim=(0, 60), ylim=(-2, 3))
    ax.set_title("joint positions")
    ax.set_xlabel("Time[s]")
    ax.set_ylabel("Angle[rad]")
    ax.grid()
    print(q1)
    plt.plot(timestamp, q1, label="q1")
    plt.plot(timestamp, q2, label="q2")
    plt.plot(timestamp, q3, label="q3")

    ax.legend()

    plt.show()


# Run simulation
animation = anim.FuncAnimation(
    fig,
    simulate,
    np.arange(0, 60, dt),
    interval=10,
    blit=True,
    init_func=init,
    repeat=False,
)
plt.show()
plot_summary()
