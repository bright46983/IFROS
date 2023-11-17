from Localization import *
import numpy as np
import rospy
from geometry_msgs.msg import Twist
class DR_3DOFDifferentialDrive(Localization):
    """
    Dead Reckoning Localization for a Differential Drive Mobile Robot.
    """
    def __init__(self, index, kSteps, robot, x0, *args):
        """
        Constructor of the :class:`prlab.DR_3DOFDifferentialDrive` class.

        :param args: Rest of arguments to be passed to the parent constructor
        """

        super().__init__(index, kSteps, robot, x0, *args)  # call parent constructor

        self.dt = 0.1  # dt is the sampling time at which we iterate the DR
        self.t_1 = 0.0  # t_1 is the previous time at which we iterated the DR
        self.wheelRadius = 0.1  # wheel radius
        self.wheelBase = 0.5  # wheel base
        self.robot.pulse_x_wheelTurns = 4096  # number of pulses per wheel turn
        rospy.init_node("ros_test")
        self.vel_sub = rospy.Subscriber('/cmd_vel',Twist, self.vel_callback)
        self.current_vel = np.array([[0],[0]])


    

    def vel_callback(self,msg):

        self.current_vel = np.array([[msg.linear.x],[msg.angular.z]])

    def Localize(self, xk_1, uk):  # motion model
        """
        Motion model for the 3DOF (:math:`x_k=[x_{k}~y_{k}~\psi_{k}]^T`) Differential Drive Mobile robot using as input the readings of the wheel encoders (:math:`u_k=[n_L~n_R]^T`).

        :parameter xk_1: previous robot pose estimate (:math:`x_{k-1}=[x_{k-1}~y_{k-1}~\psi_{k-1}]^T`)
        :parameter uk: input vector (:math:`u_k=[u_{k}~v_{k}~w_{k}~r_{k}]^T`)
        :return xk: current robot pose estimate (:math:`x_k=[x_{k}~y_{k}~\psi_{k}]^T`)
        """

        # Store previous state and input for Logging purposes
        self.etak_1 = xk_1  # store previous state
        self.uk = uk  # store input
   
        # TODO: to be completed by the student
        d_L = uk[0] * 2 * np.pi * self.wheelRadius / self.robot.pulse_x_wheelTurns
        d_R = uk[1] * 2 * np.pi * self.wheelRadius / self.robot.pulse_x_wheelTurns

        v_L = d_L / self.dt
        v_R = d_R /self.dt

        vx = (v_L + v_R)/2
        w = (v_R - v_L)/self.wheelBase
        v = np.array([[vx],[0],[w]])

        xk = xk_1.oplus(v*self.dt)
        
        
        return xk

    def GetInput(self):
        """
        Get the input for the motion model. In this case, the input is the readings from both wheel encoders.

        :return: uk:  input vector (:math:`u_k=[n_L~n_R]^T`)
        """

        # TODO: to be completed by the student
        uk,_ = self.robot.ReadEncoders()
        return uk
        

    def LocalizationLoop(self, x0, usk):
        xk_1 = x0
        xsk_1 = self.robot.xsk_1

        for self.k in range(self.kSteps):
            xsk = self.robot.fs(xsk_1, self.current_vel)  # Simulate the robot motion
            print(self.current_vel)
            uk = self.GetInput()  # Get the input from the robot
            self.xk = self.Localize(xk_1,uk)  # Localize the robot

            xsk_1 = xsk  # current state becomes previous state for next iteration
            xk_1 = self.xk

            self.PlotTrajectory()  # plot the estimated trajectory

        plt.show()
        return