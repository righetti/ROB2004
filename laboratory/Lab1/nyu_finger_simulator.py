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
import os.path

class NYUFingerSimulator:
    def __init__(self):
        # the step time
        self.dt = 0.001
        
        # Connet to pybullet and setup simulation parameters.
        p.connect(p.GUI)
        p.setGravity(0, 0, -9.81)
        p.setPhysicsEngineParameter(fixedTimeStep=self.dt, numSubSteps=1)

        # Zoom onto the robot.
        p.resetDebugVisualizerCamera(1.0, 50, -35, (0., 0., 0.))

        # Disable the gui controller as we don't use them.
        p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
        
        ###
        # Load the finger robot
        robotStartPos = [0.,0,.0]
        robotStartOrientation = p.getQuaternionFromEuler([0,0,0])

        urdf_path = './urdf/fingeredu.urdf'

        self.robotId = p.loadURDF(urdf_path, robotStartPos,
                robotStartOrientation, flags=p.URDF_USE_INERTIA_FROM_FILE,
                useFixedBase=True)
        
        # names of the joints of interest
        self.joint_names = [
            'finger_base_to_upper_joint', 
            'finger_upper_to_middle_joint', 
            'finger_middle_to_lower_joint',
        ]
        self.nj = len(self.joint_names)

        # get the proper map for the joints of interest
        bullet_joint_map = {}
        for ji in range(p.getNumJoints(self.robotId)):
            bullet_joint_map[p.getJointInfo(self.robotId, ji)[1].decode('UTF-8')] = ji

        self.bullet_joint_ids = np.array([bullet_joint_map[name] for name in self.joint_names])

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
                              q[i], 0.)
        
    def send_joint_torque(self, tau):
        """
        sends torque commands (needs an array of desired joint torques)
        """
        assert(tau.shape[0] == self.nj)
        zeroGains = tau.shape[0] * (0.,)
        
        joint_states = p.getJointStates(self.robotId, self.bullet_joint_ids)
        dq = np.zeros([self.nj])
        for i in range(self.nj):  
            dq[i] = joint_states[i][1]

        tau_sent = tau - dq * 0.1

        p.setJointMotorControlArray(self.robotId, self.bullet_joint_ids, 
                                    p.TORQUE_CONTROL, forces=tau_sent, 
                                    positionGains=zeroGains, velocityGains=zeroGains)
        
    def step(self):
        """
        does one step in the simulation (1 millisecond)
        """
        time.sleep(self.dt)
        p.stepSimulation()


