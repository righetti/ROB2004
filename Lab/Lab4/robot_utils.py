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
    
def getAdjoint(T):
    """
    returns the adjoint of a homogeneous transform
    """
    ad = np.zeros([6,6])
    ad[0:3,0:3] = T[0:3,0:3]
    ad[3:6,3:6] = T[0:3,0:3]
    ad[3:6,0:3] = vec_to_skew(T[0:3,3]) @ T[0:3,0:3]
    return ad

def translate_then_rotate_x(tran, theta):
    """
    rotation around x
    """
    T = np.eye(4)
    T[0:3,3] = tran
    T[1,1] = np.cos(theta)
    T[2,2] = T[1,1]
    T[2,1] = np.sin(theta)
    T[1,2] = -T[2,1]
    return T

def translate_then_rotate_z(tran, theta):
    """
    rotation around x
    """
    T = np.eye(4)
    T[0:3,3] = tran
    T[0,0] = np.cos(theta)
    T[1,1] = T[0,0]
    T[1,0] = np.sin(theta)
    T[0,1] = -T[1,0]
    return T

def inverseT(T):
    """
    computes the inverse of a homogeneous transform
    """
    T_inv = np.eye(4)
    T_inv[0:3,0:3] = T[0:3,0:3].T
    T_inv[0:3,3] = -T[0:3,0:3].T @ T[0:3,3]
    return T_inv

def getBodyJacobianOrientedLikeSpatialFrame(q):
    """
    this fonction returns the body Jacobian of the end-effector
    """
    l0 = 0.3
    l1 = 0.16
    l2 = 0.16
    l3 = 0.014
        
    rot_axis1 = np.array([1.,0,0,0,0,0])
    rot_axis2 = np.array([0.,0,1.,0,0,0])
    rot_axis3 = np.array([0.,0,1.,0,0,0])
        
    bodyJ = np.zeros([6,3])
        
    # first we compute the forward kinematics
    TS_H1 = translate_then_rotate_x(np.array([l0,0,0]), q[0])
    TH1_H2 = translate_then_rotate_z(np.array([0,0,l3]), q[1])
    TH2_K = translate_then_rotate_z(np.array([0,-l1,0]), q[2])
    TK_F = translate(np.array([0,-l2,0]))

    TS_F_notranslate = TS_H1 @ TH1_H2 @ TH2_K @ TK_F
    TS_F_notranslate[0:3,3] = np.zeros([3])
        
    # now we compute the pose of all the frames with respect to F
    TF_K = inverseT(TK_F)
    TF_H2 = TF_K @ inverseT(TH2_K)
    TF_H1 = TF_H2 @ inverseT(TH1_H2)
        
    # we compute the body Jacobian
    bodyJ[:,0] = getAdjoint(TF_H1) @ rot_axis1
    bodyJ[:,1] = getAdjoint(TF_H2) @ rot_axis2
    bodyJ[:,2] = getAdjoint(TF_K) @ rot_axis3

    return (getAdjoint(TS_F_notranslate) @ bodyJ)[3:,:]