from lab5_robotics import *



class MobileManipulator:
    '''
        Constructor.

        Arguments:
        d (Numpy array): list of displacements along Z-axis
        theta (Numpy array): list of rotations around Z-axis
        a (Numpy array): list of displacements along X-axis
        alpha (Numpy array): list of rotations around X-axis
        revolute (list of Bool): list of flags specifying if the corresponding joint is a revolute joint
    '''
    def __init__(self, d, theta, a, alpha, revolute):
        self.d = d # d of each manipulator's link
        self.theta = theta # theta of each manipulator's joint
        self.a = a
        self.alpha = alpha
        self.revolute = revolute
        self.revoluteExt = [True,False] + self.revolute  # List of joint types extended with base joints
        self.r = 0            # Distance from robot centre to manipulator base
        self.dof = len(self.revoluteExt) # Number of DOF of the system
        self.q = np.zeros((len(self.revolute),1)) # Vector of joint positions (manipulator)
        self.eta = np.zeros((3,1)) # Vector of base pose (position & orientation)
        self.update(np.zeros((self.dof,1)), 0.0) # Initialise robot state

    '''
        Method that updates the state of the robot.

        Arguments:
        dQ (Numpy array): a column vector of quasi velocities
        dt (double): sampling time
    '''
    def update(self, dQ, dt, priority = "ROTATE"):
        # Update manipulator
        self.q += dQ[2:, 0].reshape(-1,1) * dt
        for i in range(len(self.revolute)):
            if self.revolute[i]:
                self.theta[i] = self.q[i]
            else:
                self.d[i] = self.q[i] 
                 
        # Update mobile base pose 
        # displacment
        d = dQ[1,0] * dt 
        theta = dQ[0,0] * dt 
        print(d)
        # previous position and orientation
        x_1 = self.eta[0,0]  
        y_1 = self.eta[1,0]
        yaw_1 = self.eta[2,0]

        if priority == "ROTATE":
            # Update orientation
            self.eta[2,0] =  yaw_1 + theta
            self.eta[0,0] =  x_1 + d*np.cos(self.eta[2,0])  #xbc oplus equation
            self.eta[1,0] =  y_1 + d*np.sin(self.eta[2,0])
            
        
        elif priority == "TRANSLATE":
            # Update position
            self.eta[0,0] =  x_1 + d*np.cos(yaw_1)  #xbc oplus equation
            self.eta[1,0] =  y_1 + d*np.sin(yaw_1)
            self.eta[2,0] =  yaw_1 + theta
            
            
        else: 
            arc_radius = dQ[1,0]/dQ[0,0]

            # Update position and orientation
            self.eta[0,0] = x_1 + arc_radius * (np.sin(yaw_1 + theta) - np.sin(yaw_1))  # Calculate new x position
            self.eta[1,0] = y_1 + arc_radius * (-np.cos(yaw_1 + theta) + np.cos(yaw_1))  # Calculate new y position
            self.eta[2,0] = yaw_1 + theta  # Update orientation

    
        #new position and orientation
        x = self.eta[0,0]
        y = self.eta[1,0]
        yaw = self.eta[2,0]
        
        # Transformation of the mobile base to manipulator
        R = np.array([[np.cos(yaw),  -np.sin(yaw), 0, 0 ],
                       [np.sin(yaw) , np.cos(yaw), 0, 0 ],
                       [0, 0, 1, 0],
                       [0, 0, 0, 1]])
        T = np.array([[1,  0, 0, x ],
                       [0 ,1, 0, y ],
                       [0, 0, 1, 0],
                       [0, 0, 0, 1]])
        iTb = np.array([[np.cos(yaw),  -np.sin(yaw), 0, x ],
                       [np.sin(yaw) , np.cos(yaw), 0, y ],
                       [0, 0, 1, 0],
                       [0, 0, 0, 1]])
        iTb = T @ R
        # Base kinematics  
        Tb = iTb 

        
        self.theta[0] -= np.pi/2

        # Combined system kinematics (DH parameters extended with base DOF)
        dExt = np.concatenate([np.array([  0 ,  self.r ]), self.d])
        thetaExt = np.concatenate([np.array([  np.pi/2 ,  0  ]), self.theta])
        aExt = np.concatenate([np.array([ 0   , 0   ]), self.a])
        alphaExt = np.concatenate([np.array([  np.pi/2   ,  -np.pi/2   ]), self.alpha])

        self.T = kinematics(dExt, thetaExt, aExt, alphaExt, Tb)

    ''' 
        Method that returns the characteristic points of the robot.
    '''
    def drawing(self):
        return robotPoints2D(self.T)

    '''
        Method that returns the end-effector Jacobian.
    '''
    def getEEJacobian(self):
        return jacobian(self.T, self.revoluteExt)

    '''
        Method that returns the end-effector transformation.
    '''
    def getEETransform(self):
        return self.T[-1]

    '''
        Method that returns the position of a selected joint.

        Argument:
        joint (integer): index of the joint

        Returns:
        (double): position of the joint
    '''
    def getJointPos(self, joint):
        return self.q[joint-2]


    '''
        Method that returns base position and orientation
    '''
    def getBasePose(self):
        return self.eta
    
    '''
        Method that returns number of DOF of the manipulator.
    '''
    def getDOF(self):
        return self.dof

    ###
    def getLinkJacobian(self, link):
        return jacobianLink(self.T, self.revoluteExt, link)

    def getLinkTransform(self, link):
        return self.T[link]

