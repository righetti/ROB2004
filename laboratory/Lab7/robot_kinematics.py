import numpy as np


def translate(vector):
    """
    returns an homogenous transform for a 2D translate of vector
    """
    transform = np.eye(4)
    transform[0:3,3] = vector
    return transform

def vec_to_skew(p):
    """
    transforms a 3D vector into a skew symmetric matrix
    """
    return np.array([[0.,-p[2],p[1]],[p[2],0.,-p[0]],[-p[1],p[0],0]])

def twist_to_skew(twist):
    """
    transforms a twist into its bracket representation
    """
    br = np.zeros([4,4])
    br[0:3,0:3] = vec_to_skew(twist[0:3])
    br[0:3,3] = twist[3:6]
    return br
    
def getAdjoint(T):
    """
    returns the adjoint of a homogeneous transform
    """
    ad = np.zeros([6,6])
    ad[0:3,0:3] = T[0:3,0:3]
    ad[3:6,3:6] = T[0:3,0:3]
    ad[3:6,0:3] = vec_to_skew(T[0:3,3]).dot(T[0:3,0:3])
    return ad

def getExpPureRotationTwist(twist, theta):
    """
    computes the exponential of a pure rotation unit twist applied for theta seconds

    we specialize the function to speed up computations for the real robot control loop
    since we know we won't need the v part, we can ignore this part
    """
    T = np.eye(4)
    br = vec_to_skew(twist[0:3])
    T[0:3,0:3] = np.eye(3) + np.sin(theta) * br + (1-np.cos(theta))*br.dot(br)
    return T

def inverseT(T):
    """
    computes the inverse of a homogeneous transform
    """
    T_inv = np.eye(4)
    T_inv[0:3,0:3] = T[0:3,0:3].T
    T_inv[0:3,3] = -T[0:3,0:3].T.dot(T[0:3,3])
    return T_inv

class robot_kinematics:
    """
    This class contains the necessary variables to compute the kinematics and Jacobians of the robot
    update_kinematics computes all the necessary quantities and homogeneous transforms and store
    them in instance variables that can be used afterwards
    """
    def __init__(self):
        # here we define the variables for the robot lengths
        self.l0 = 0.3
        self.l1 = 0.16
        self.l2 = 0.16
        self.l3 = 0.014
        
        self.rot_axis1 = np.array([1.,0,0,0,0,0])
        self.rot_axis2 = np.array([0.,0,1,0,0,0])
        self.rot_axis3 = np.array([0.,0,1,0,0,0])
        
        self.link1_xcom = translate(np.array([0.3-0.079, 0, 0]))
        self.link1_mass = 0.14854
        self.link2_xcom = translate(np.array([0.,-0.079,0.019]))
        self.link2_mass = 0.14854
        self.link3_xcom = translate(np.array([0,-0.089,0.009]))
        self.link3_mass = 0.0307
        self.finger_xcom = translate(np.array([0,0,0.]))
        self.finger_mass = 0.01

        self.q = np.array([0,0,0]) 

        self.spatialJ = np.zeros([6,3])
        self.bodyJ = np.zeros([6,3])
        self.orientedJ = np.zeros([6,3])
        
        self.update_kinematics(self.q)
        
    def update_kinematics(self, q):
        """
        Receives as input the state of the robot (vector of 3 angles) and computes all the homogeneous transforms
        and the Jacobians
        """
        # first we compute the homogeneous transforms for the relative frames using theta0 to theta2
        self.TS_H1 = translate(np.array([self.l0,0,0])).dot(getExpPureRotationTwist(self.rot_axis1,q[0]))
        self.TH1_H2 = translate(np.array([0,0,self.l3])).dot(getExpPureRotationTwist(self.rot_axis2,q[1]))
        self.TH2_K = translate(np.array([0,-self.l1,0])).dot(getExpPureRotationTwist(self.rot_axis3,q[2]))
        self.TK_F = translate(np.array([0,-self.l2,0]))
        
        # now we compute the pose of all the frames with respect to S
        self.TS_H2 = self.TS_H1.dot(self.TH1_H2)
        self.TS_K = self.TS_H2.dot(self.TH2_K)        
        self.TS_F = self.TS_K.dot(self.TK_F)
        
        # we compute the spatial Jacobian
        self.spatialJ[:,0] = getAdjoint(self.TS_H1).dot(self.rot_axis1)
        self.spatialJ[:,1] = getAdjoint(self.TS_H2).dot(self.rot_axis2)
        self.spatialJ[:,2] = getAdjoint(self.TS_K).dot(self.rot_axis3)
        
        # the body Jacobian
        self.bodyJ = getAdjoint(inverseT(self.TS_F)).dot(self.spatialJ)

        # and the Jacobian with respect to O (the one we will use)
        TO_S = np.eye(4)
        TO_S[0:3,3] = -self.TS_F[0:3,3]
        self.orientedJ = getAdjoint(TO_S).dot(self.spatialJ)

    def getG(self):
        """
        returns the gravity vector g(theta) which is the generalized force applied on
        the joints due to the weight of each link of the robot
        this assumes that update_kinematics was called before
        """
        g = 9.81

        g_link1 = np.array([0,0,0,0,-self.link1_mass * g, 0])
        g_link1[3:] = self.TS_H1[0:3,0:3].T.dot(g_link1[3:])
        g_link1_in_S = getAdjoint(inverseT(self.TS_H1.dot(self.link1_xcom))).T.dot(g_link1)

        g_link2 = np.array([0,0,0,0,-self.link2_mass * g, 0])
        g_link2[3:] = self.TS_H2[0:3,0:3].T.dot(g_link2[3:])
        g_link2_in_S = getAdjoint(inverseT(self.TS_H2.dot(self.link2_xcom))).T.dot(g_link2)

        g_link3 = np.array([0,0,0,0,-self.link3_mass * g, 0])
        g_link3[3:] = self.TS_K[0:3,0:3].T.dot(g_link3[3:])
        g_link3_in_S = getAdjoint(inverseT(self.TS_K.dot(self.link3_xcom))).T.dot(g_link3)

        g_finger = np.array([0,0,0,0,-self.finger_mass * g, 0])
        g_finger[3:] = self.TS_F[0:3,0:3].T.dot(g_finger[3:])
        g_finger_in_S = getAdjoint(inverseT(self.TS_F.dot(self.finger_xcom))).T.dot(g_finger)

        tau = np.zeros([3])
        tau[0] = self.spatialJ[:,0].dot(g_link1_in_S + g_link2_in_S + g_link3_in_S + g_finger_in_S)
        tau[1] = self.spatialJ[:,1].dot(g_link2_in_S + g_link3_in_S + g_finger_in_S)
        tau[2] = self.spatialJ[:,2].dot(g_link3_in_S + g_finger_in_S)
        return tau


def analytic_inverse_kinematics2D(x,y):
    """
    inverse kinematics function
    input (x,y) position of the foot
    output a list of 2D vectors which are possible solutions to the problem 
    (the list is empty if there are no solutions)
    """
    
    l0 = 0.3
    l1 = 0.16
    l2 = 0.16
    
    l_des = np.sqrt((x-l0)**2 + y**2)
    
    ###First we check that the target is feasible otherwise we return empty
    if l_des > l1 + l2:
        # this is impossible, there are no solutions we return an empty list
        return []

    # we compute the two possible solutions for theta2
    # note that if l_des == l1 + l2 then theta2_p = theta2_m = 0
    # so we will return twice the same solution (not ideal but simpler)
    theta2_p = np.arccos((l_des**2 - l1**2 - l2**2)/(2*l1*l2))
    theta2_m = - theta2_p
    
    # we now compute alpha and beta as defined above
    alpha = np.arccos((-l2**2 + l1**2 + l_des**2)/(2*l1*l_des))
    beta = np.arctan2(y,x-l0)
    
    # we compute alpha1 (the 2 possibilities)
    theta1_p = np.pi/2 - alpha + beta
    theta1_m = (alpha + beta + np.pi/2)
    
    # we return a list that contains the 2 solutions
    return [np.array([theta1_p, theta2_p]), np.array([theta1_m, theta2_m])]

def compute_trajectory(th_init, th_goal, movement_duration, t):
    """ 
    trajectory generation function - returns the desired position and velocity
    at time t for a trajectory of duration movement_duration starting
    at t=0 at th_init and stopping at t=movement_duration at th_goal

    We use a 5th order polynominal to ensure that the trajectory starts and ends
    with 0 velocity and acceleration
    """
    # first we compute the coefficients (as notes above)
    a5 = 6/(movement_duration**5)
    a4 = -15/(movement_duration**4)
    a3 = 10/(movement_duration**3)
    
    # now we compute s and ds/dt
    s = a3 * t**3 + a4 * t**4 + a5 * t**5
    ds = 3 * a3 * t**2 + 4 * a4 * t**3 + 5 * a5 * t**4
    
    #now we compute th and dth/dt (the angle and its velocity)
    th = th_init + s * (th_goal - th_init)
    dth = (th_goal - th_init) * ds
    
    # we return the answer
    return th, dth