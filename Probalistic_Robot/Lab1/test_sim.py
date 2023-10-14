from DifferentialDriveSimulatedRobot import DifferentialDriveSimulatedRobot
import numpy as np
import time
import math


def moveCircle(desired_vel_x, radius):
    desired_w_vel  = (desired_vel_x / radius)
    desired_vel = np.array([[desired_vel_x],[desired_w_vel]])
    return desired_vel

def move8( desired_vel_x, radius,robot,previous_vel):
    if previous_vel[0] != 0:
        desired_vel = previous_vel
        if robot.xsk[0]  * robot.xsk_1[0] < 0.0 and robot.xsk[0] < robot.xsk_1[0]: 
            desired_vel[1] = -previous_vel[1]
    else:
        desired_w_vel  = (desired_vel_x / radius)
        desired_vel = np.array([[desired_vel_x],[desired_w_vel]])
    
    return desired_vel



if __name__== '__main__':
    current_state = np.zeros((6,1))
    previous_vel = np.array([[0],[0]])
    desired_vel_x = 0.5
    desired_radius = 20
    M2D = [np.array([[-40, 5]]).T,
           np.array([[-5, 40]]).T,
           np.array([[-5, 25]]).T,
           np.array([[-3, 50]]).T,
           np.array([[-20, 3]]).T,
           np.array([[40,-40]]).T]
    robot = DifferentialDriveSimulatedRobot(current_state,M2D)
    kSteps = 100000000
    try:
        for i in range(kSteps):
            desired_vel = moveCircle(desired_vel_x,desired_radius)
            #desired_vel = move8(desired_vel_x,desired_radius,robot,previous_vel)
            current_state = robot.fs(current_state,desired_vel)
            previous_vel = desired_vel
            time.sleep(0.0001)
    except KeyboardInterrupt:
        print('interrupted!')