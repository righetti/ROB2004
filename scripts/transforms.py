import numpy as np
from numpy import cos, sin, pi, exp, log

#takes a vector [w] and returns the corresponding 3x3 skew symmetric matrix W
#such that for any other vector v, we have (w x v) = W.v
def to_skew(w):
    return np.array([[0, -w[2], w[1]],[w[2],0,-w[0]],[-w[1], w[0],0]])

#rotation around x (Returns 3x3 matrix)
def Rx(th):
    return np.array([[1,0,0],
                     [0,cos(th),-sin(th)],
                     [0,sin(th), cos(th)]])

#rotation around y (returns 3x3 matrix)
def Ry(th):
    return np.array([[cos(th), 0, sin(th)],
                    [0,1,0],
                    [-sin(th),0,cos(th)]])

#rotation around z (returns 3x3 matrix)
def Rz(th):
    return np.array([[cos(th), -sin(th), 0],
                     [sin(th),cos(th),0],
                     [0,0,1]])

#exponential map from [w] to 3x3 rotation matrix
def R_exp(w):
    w_hat = to_skew(w)
    norm = np.linalg.norm(w)
    if(norm > 0.001):
        return np.eye(3) + w_hat /norm * sin(norm) + w_hat.dot(w_hat)/(norm**2)*(1-cos(norm))
    else:
        return np.eye(3)

#exponential map from [v,w] to 4x4 homogenous transform
def T_exp(v,w):
    R = R_exp(w)
    T = np.eye(4)
    if(np.linalg.norm(w)<0.001):
        T[0:3,3] = v
    else:
        w_hat = to_skew(w)
        w_norm = np.linalg.norm(w)
        A = np.eye(3) + (1 - cos(w_norm))/(w_norm**2) * w_hat + (w_norm - sin(w_norm))/(w_norm**3) * w_hat.dot(w_hat)
        t = A.dot(v)
        T[0:3,0:3] = R
        T[0:3,3] = t
    return T

def hom(R,p):
    T = np.identity(4)
    T[0:3,0:3] = R
    T[0:3,3] = p
    return T

#returns the adjoint of homogeneous transform T
def adjoint(T):
    ad = np.zeros([6,6])
    ad[0:3,0:3] = T[0:3,0:3]
    ad[3:6,3:6] = T[0:3,0:3]
    ad[0:3,3:6] = to_skew(T[0:3,3]).dot(T[0:3,0:3])
    return ad

    