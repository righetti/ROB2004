#///////////////////////////////////////////////////////////////////////////////
#// BSD 3-Clause License
#//
#// Copyright (C) 2020, New York University
#// Copyright note valid unless otherwise stated in individual files.
#// All rights reserved.
#///////////////////////////////////////////////////////////////////////////////

import numpy as np
import pybullet as p


import time
import os

from scipy.linalg import expm

def expSE3(twist):
    bracket_twist = np.zeros([4,4])
    bracket_twist[0:3,0:3] = vec_to_skew(twist[0:3])
    bracket_twist[0:3,3] = twist[3:6]
    T = expm(bracket_twist)
    return T

def inverseT(T):
    """
    computes the inverse of a homogeneous transform
    """
    T_inv = np.eye(4)
    T_inv[0:3,0:3] = T[0:3,0:3].T
    T_inv[0:3,3] = -T[0:3,0:3].T.dot(T[0:3,3])
    return T_inv

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
    ad[3:6,0:3] = vec_to_skew(T[0:3,3]).dot(T[0:3,0:3])
    return ad


class joint:
    def __init__(self, T0, axis, parent, xcom, mass):
        self.T0 = T0.copy()
        self.axis = axis
        self.parent=parent
        self.currentT = T0.copy()
        self.parentT = np.eye(4)
        self.G = np.eye(6)
        self.G[3:6,3:6] = mass * np.eye(3)
        xcomT = np.eye(4)
        xcomT[0:3,3] = xcom
        self.G = getAdjoint(inverseT(xcomT)).T @ self.G @ getAdjoint(inverseT(xcomT))

    def updateKin(self, theta):
        if self.parent:
            self.parentT = self.T0 @ expSE3(self.axis*theta)
            self.currentT = self.parent.currentT @ self.parentT
        else:
            self.currentT = self.T0 @ expSE3(self.axis*theta)        
        
class PandaRobot:
    def __init__(self):
        self._joints = []
        
        # joint 1
#         T = np.eye(4)
        orig_or = np.pi/2.
        T = np.array([[np.cos(orig_or), -np.sin(orig_or), 0,0],
                     [np.sin(orig_or),np.cos(orig_or),0,0],
                     [0,0,1,0.333],
                    [0,0,0,1]])
#         T[2,3] = 0.333
        self._joints.append(joint(T, np.array([0,0,1,0,0,0]), None, np.array([0,0,0]), 3.06+2.34))
        
        # joint 2
        T = np.array([[1,0,0,0],
                     [0,np.cos(-np.pi/2.),-np.sin(-np.pi/2.),0],
                     [0,np.sin(-np.pi/2), np.cos(-np.pi/2.),0],
                    [0,0,0,1]])
        self._joints.append(joint(T, np.array([0,0,1,0,0,0]), self._joints[0], np.array([0,0,0]), 2.36))
        
        # joint 3
        T = np.array([[1,0,0,0],
                     [0,np.cos(np.pi/2.),-np.sin(np.pi/2.),-0.316],
                     [0,np.sin(np.pi/2.), np.cos(np.pi/2.),0],
                    [0,0,0,1]])
        self._joints.append(joint(T, np.array([0,0,1,0,0,0]), self._joints[1], np.array([0, 0,0]), 2.38))
        
        # joint 4
        T = np.array([[1,0,0,0.0825],
                     [0,np.cos(np.pi/2.),-np.sin(np.pi/2.),0],
                     [0,np.sin(np.pi/2.), np.cos(np.pi/2.),0],
                    [0,0,0,1]])
        self._joints.append(joint(T, np.array([0,0,1,0,0,0]), self._joints[2], np.array([0, 0,0]), 2.43))
        
        # joint 5
        T = np.array([[1,0,0,-0.0825],
                     [0,np.cos(-np.pi/2.),-np.sin(-np.pi/2.),0.384],
                     [0,np.sin(-np.pi/2.), np.cos(-np.pi/2.),0],
                    [0,0,0,1]])
        self._joints.append(joint(T, np.array([0,0,1,0,0,0]), self._joints[3], np.array([0,0,0]), 3.5))
        
        # joint 6
        T = np.array([[1,0,0,0],
                     [0,np.cos(np.pi/2.),-np.sin(np.pi/2.),0],
                     [0,np.sin(np.pi/2.), np.cos(np.pi/2.),0],
                    [0,0,0,1]])
        self._joints.append(joint(T, np.array([0,0,1,0,0,0]), self._joints[4], np.array([0, 0,0]), 1.47))
        
        # joint 7
        T = np.array([[1,0,0,0.088],
                     [0,np.cos(np.pi/2.),-np.sin(np.pi/2.),0],
                     [0,np.sin(np.pi/2.), np.cos(np.pi/2.),0],
                    [0,0,0,1]])
        self._joints.append(joint(T, np.array([0,0,1,0,0,0]), self._joints[5], np.array([0, 0, 0.107*0.68/(0.68+0.45)]), 0.45+0.68))
        
        self._T0endeff = np.array([[np.cos(-0.785398163397), -np.sin(-0.785398163397), 0,0],
                     [np.sin(-0.785398163397),np.cos(-0.785398163397),0,0],
                     [0,0,1,0.107],
                    [0,0,0,1]])
        
        self.FK(np.zeros([7]))
        self._spacejacobian = np.zeros([6,7])
        
    def FK(self, q):
        for i in range(7):
            self._joints[i].updateKin(q[i])
        return self._joints[6].currentT @ self._T0endeff
        
    def get_jacobian(self, q, jac_frame='S'):
        Tendeff = self.FK(q)
        for i in range(7):
            self._spacejacobian[:,i] = getAdjoint(self._joints[i].currentT) @ self._joints[i].axis
        
        if jac_frame == 'S':
            return self._spacejacobian
        elif jac_frame == 'B':
            return getAdjoint(inverseT(Tendeff)) @ self._spacejacobian
        elif jac_frame == 'O':
            Tendeff[0:3,0:3] = np.eye(3)
            return getAdjoint(inverseT(Tendeff)) @ self._spacejacobian
        else:
            print('invalid request')
            raise Exception('invalid Jacobian request')
            
    def g(self, q):
        Tendeff = self.FK(q)
        dV = np.zeros([6,7])
        dV[5,0] = 9.81
        F = np.zeros([6,7])
        tau = np.zeros(7)
        # forward pass
        for i in range(6):
            dV[:,i+1] = getAdjoint(inverseT(self._joints[i+1].parentT)) @ dV[:,i]
            
        # backward pass
        for i in range(6,-1,-1):
            if i == 6:
                F[:,i] = self._joints[i].G @ dV[:,i]
            else:
                F[:,i] = self._joints[i].G @ dV[:,i] + getAdjoint(inverseT(self._joints[i+1].parentT)).T @ F[:,i+1]
            tau[i] = np.dot(F[:,i], self._joints[i].axis)
                            
        return tau
            

class Simulator:
    def __init__(self):
        # the step time
        self.dt = 0.005
        
        # Connet to pybullet and setup simulation parameters.
        p.connect(p.GUI)
        p.setGravity(0, 0, -9.81)
        p.setPhysicsEngineParameter(fixedTimeStep=self.dt, numSubSteps=1)

        # Zoom onto the robot.
        p.resetDebugVisualizerCamera(2.0, 180, -30, (0., 1., 0.6))

        # Disable the gui controller as we don't use them.
        p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
        
        ## create the world
        plane = p.loadURDF("./urdf/plane.urdf")
        p.resetBasePositionAndOrientation(plane, [0,0,0.], (0., 0., 0., 1.))
        
        ## add the table
        table = p.loadURDF("./urdf/table.urdf")
        p.resetBasePositionAndOrientation(table, [0,0.6,0.], (0., 0., 0., 1.))
        
        ## add the traybox
        bowl = p.loadURDF("./urdf/bowl.urdf")
        p.resetBasePositionAndOrientation(bowl, [-0.3,0.55,0.65], (0., 0., 0., 1.))
        
        cube = p.loadURDF("./urdf/block.urdf")
        p.resetBasePositionAndOrientation(cube, [0.35,0.58,0.65], (0., 0., 0., 1.))
        
        cube = p.loadURDF("./urdf/block.urdf")
        p.resetBasePositionAndOrientation(cube, [0.15,0.67,0.65], (0., 0., 0., 1.))
        
        ###
        # Load the finger robot
        robotStartPos = [0, 0, 0.5]
        robotStartOrientation = p.getQuaternionFromEuler([0,0,np.pi/2.])

#         urdf_path = './urdf/iiwa.urdf'
        urdf_path = "./urdf/panda.urdf"

        self.robotId = p.loadURDF(urdf_path, robotStartPos,
                robotStartOrientation, flags=p.URDF_USE_INERTIA_FROM_FILE,
                useFixedBase=True)
        
        # names of the joints of interest
        self.joint_names = [
            'panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7'
        ]
        self.gripper_name = ['panda_finger_joint1', 'panda_finger_joint2']
        
        self.nj = len(self.joint_names)

        # get the proper map for the joints of interest
        bullet_joint_map = {}
        for ji in range(p.getNumJoints(self.robotId)):
            bullet_joint_map[p.getJointInfo(self.robotId, ji)[1].decode('UTF-8')] = ji

        self.bullet_joint_ids = np.array([bullet_joint_map[name] for name in self.joint_names])
        self.bullet_gripper_ids = np.array([bullet_joint_map[name] for name in self.gripper_name])
        

        # Disable the velocity control on the joints as we use torque control.
        p.setJointMotorControlArray(self.robotId, self.bullet_joint_ids,
                                    p.VELOCITY_CONTROL, forces=np.zeros(self.nj))
        
    def get_state(self):
        """ 
        returns 2 arrays: joint positions and velocities
        """
        joint_states = p.getJointStates(self.robotId, self.bullet_joint_ids)
        q = np.zeros([self.nj])
        dq = np.zeros_like(q)
        for i in range(self.nj):  
            q[i] = joint_states[i][0]
            dq[i] = joint_states[i][1]
        return q, dq
    
    def reset_state(self, q):
        """
        reset the simulation to the desired position p (as array)
        """
        for i, bullet_joint_id in enumerate(self.bullet_joint_ids):
            p.resetJointState(self.robotId, bullet_joint_id,
                              q[i], 0.1)
        
    def send_joint_torque(self, tau):
        """
        sends torque commands (needs an array of desired joint torques)
        """
        assert(tau.shape[0] == self.nj)
        zeroGains = tau.shape[0] * (0.,)

        p.setJointMotorControlArray(self.robotId, self.bullet_joint_ids, 
                                    p.TORQUE_CONTROL, forces=tau, 
                                    positionGains=zeroGains, velocityGains=zeroGains)
        
    def gripper_move(self, pos):
        p.setJointMotorControlArray(self.robotId, self.bullet_gripper_ids, 
                                    p.POSITION_CONTROL, pos)
                
    def step(self):
        """
        does one step in the simulation (1 millisecond)
        """
        time.sleep(self.dt)
        p.stepSimulation()
