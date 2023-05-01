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
    ad[3:6,0:3] = vec_to_skew(T[0:3,3]) @ T[0:3,0:3]
    return ad

def getExpPureRotationTwist(twist, theta):
    """
    computes the exponential of a pure rotation unit twist applied for theta seconds

    we specialize the function to speed up computations for the real robot control loop
    since we know we won't need the v part, we can ignore this part
    """
    T = np.eye(4)
    br = vec_to_skew(twist[0:3])
    T[0:3,0:3] = np.eye(3) + np.sin(theta) * br + (1-np.cos(theta))*br @ br
    return T

def inverseT(T):
    """
    computes the inverse of a homogeneous transform
    """
    T_inv = np.eye(4)
    T_inv[0:3,0:3] = T[0:3,0:3].T
    T_inv[0:3,3] = -T[0:3,0:3].T @ T[0:3,3]
    return T_inv

def getGravity(q):
    l0 = 0.3
    l1 = 0.16
    l2 = 0.16
    l3 = 0.014
        
    rot_axis1 = np.array([1.,0,0,0,0,0])
    rot_axis2 = np.array([0.,0,1.,0,0,0])
    rot_axis3 = np.array([0.,0,1.,0,0,0])
        
    link1_xcom = translate(np.array([0.3-0.079, 0, 0]))
    link1_mass = 0.14854
    link2_xcom = translate(np.array([0.,-0.079,0.019]))
    link2_mass = 0.14854
    link3_xcom = translate(np.array([0,-0.089,0.009]))
    link3_mass = 0.0307
    finger_xcom = translate(np.array([0,0,0.]))
    finger_mass = 0.01

    spatialJ = np.zeros([6,3])
        
    # first we compute the homogeneous transforms for the relative frames using theta0 to theta2
    TS_H1 = translate(np.array([l0,0,0])) @ getExpPureRotationTwist(rot_axis1,q[0])
    TH1_H2 = translate(np.array([0,0,l3])) @ getExpPureRotationTwist(rot_axis2,q[1])
    TH2_K = translate(np.array([0,-l1,0])) @ getExpPureRotationTwist(rot_axis3,q[2])
    TK_F = translate(np.array([0,-l2,0]))
        
    # now we compute the pose of all the frames with respect to S
    TS_H2 = TS_H1 @ TH1_H2
    TS_K = TS_H2 @ TH2_K        
    TS_F = TS_K @ TK_F
        
    # we compute the spatial Jacobian
    spatialJ[:,0] = getAdjoint(TS_H1) @ rot_axis1
    spatialJ[:,1] = getAdjoint(TS_H2) @ rot_axis2
    spatialJ[:,2] = getAdjoint(TS_K) @ rot_axis3

    # now we compute G
    g = -9.81

    g_link1 = np.array([0,0,0,0,-link1_mass * g, 0])
    g_link1[3:] = TS_H1[0:3,0:3].T @ g_link1[3:]
    g_link1_in_S = getAdjoint(inverseT(TS_H1 @ link1_xcom)).T @ g_link1

    g_link2 = np.array([0,0,0,0,-link2_mass * g, 0])
    g_link2[3:] = TS_H2[0:3,0:3].T @ g_link2[3:]
    g_link2_in_S = getAdjoint(inverseT(TS_H2 @ link2_xcom)).T @ g_link2

    g_link3 = np.array([0,0,0,0,-link3_mass * g, 0])
    g_link3[3:] = TS_K[0:3,0:3].T @ g_link3[3:]
    g_link3_in_S = getAdjoint(inverseT(TS_K @ link3_xcom)).T @ g_link3

    g_finger = np.array([0,0,0,0,-finger_mass * g, 0])
    g_finger[3:] = TS_F[0:3,0:3].T @ g_finger[3:]
    g_finger_in_S = getAdjoint(inverseT(TS_F @ finger_xcom)).T @ g_finger

    tau = np.zeros([3])
    tau[0] = spatialJ[:,0] @ (g_link1_in_S + g_link2_in_S + g_link3_in_S + g_finger_in_S)
    tau[1] = spatialJ[:,1] @ (g_link2_in_S + g_link3_in_S + g_finger_in_S)
    tau[2] = spatialJ[:,2] @ (g_link3_in_S + g_finger_in_S)
    return tau