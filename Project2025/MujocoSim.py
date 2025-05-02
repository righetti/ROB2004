import time
from copy import deepcopy
import mujoco
import mujoco.viewer
import numpy as np
import os

import pinocchio as pin
from pinocchio import RobotWrapper

ASSETS_PATH = os.path.join(os.path.dirname(__file__), 'assets')
END_EFF_FRAME_ID = 19

class FR3Sim:
    def __init__(self, interface_type = 'torque', render=True, dt=0.001, xml_path=None):
        assert interface_type in ['torque'], 'The interface should be velocity or torque'
        self.interface_type = interface_type
        if xml_path is not None:
            self.model = mujoco.MjModel.from_xml_path(xml_path)
        else:
            self.model = mujoco.MjModel.from_xml_path(
                os.path.join(ASSETS_PATH,'fr3_on_table.xml')
            )
        self.simulated = True
        self.data = mujoco.MjData(self.model)
        self.dt = dt
        _render_dt = 1/60
        self.render_ds_ratio = max(1, _render_dt//dt)

        if render:
            self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
            self.render = True
            self.viewer.cam.distance = 3.0
            self.viewer.cam.azimuth = 90
            self.viewer.cam.elevation = -45
            self.viewer.cam.lookat[:] = np.array([0.0, -0.25, 0.824])
        else:
            self.render = False

        self.model.opt.gravity[2] = -9.81
        self.model.opt.timestep = dt
        self.step_counter = 0

        self.joint_names = [
            "joint1",
            "joint2",
            "joint3",
            "joint4",
            "joint5",
            "joint6",
            "joint7",
        ]

        self.q0 = np.array([0.0, -0.785398163, 0.0, -2.35619449, 0.0, 1.57079632679, 0.785398163397])
        self.reset()
        mujoco.mj_step(self.model, self.data)
        self.viewer.sync()
        self.nv = self.model.nv
        self.jacp = np.zeros((3, self.nv))
        self.jacr = np.zeros((3, self.nv))
        self.M = np.zeros((self.nv, self.nv))
        self.latest_command_stamp = time.time()
        self.actuator_tau = np.zeros(7)
        self.tau_ff = np.zeros(7)
        self.dq_des = np.zeros(7)

        urdf = os.path.join(ASSETS_PATH, "fr3.urdf")
        meshes_dir = os.path.join(ASSETS_PATH, "meshes")
        self.pin_model = RobotWrapper.BuildFromURDF(urdf, meshes_dir)

    def reset(self):
        self.data.qpos[:7] = self.q0
        self.data.qvel[:7] = np.zeros(7)
        mujoco.mj_step(self.model, self.data)
        # Render every render_ds_ratio steps (60Hz GUI update)
        if self.render and (self.step_counter%self.render_ds_ratio)==0:
            self.viewer.sync()

    def get_state(self):
        return self.data.qpos[:7], self.data.qvel[:7]
        # return {"q":self.data.qpos[:7], 
        #        "dq":self.data.qpos[:7],
        #        'tau_est':(self.data.qfrc_constraint.squeeze()+self.data.qfrc_smooth.squeeze())[0:7]}

    def send_joint_torque(self, torques, finger_pos=None):
        self.tau_ff = torques
        self.latest_command_stamp = time.time()
        self.step(finger_pos)
        
    def step(self, finger_pos=None):
        tau = self.tau_ff
        self.actuator_tau = tau

        if finger_pos is not None:
            tau = np.append(tau, finger_pos)
            self.data.ctrl[:8] = tau.squeeze()
        else:
            self.data.ctrl[:7] = tau.squeeze()
        self.step_counter += 1
        mujoco.mj_step(self.model, self.data)
        # Render every render_ds_ratio steps (60Hz GUI update)
        if self.render and (self.step_counter%self.render_ds_ratio)==0:
            self.viewer.sync()

    def get_gravity(self, q):
        g = self.pin_model.gravity(np.append(q,[0.,0.]))
        return g[:7]

    def get_jacobian(self, q):
        J_temp = self.pin_model.computeFrameJacobian(np.append(q,[0.,0.]), END_EFF_FRAME_ID)
        J = np.zeros([6,7])
        J[3:6,:] = J_temp[0:3,:7]
        J[0:3,:] = J_temp[3:6,:7]
        return J[:,:7]
    
    def get_pose(self, q):
        T_S_F = self.pin_model.framePlacement(np.append(q,[0.,0.]), END_EFF_FRAME_ID)
        return T_S_F.homogeneous

    def close(self):
        self.viewer.close()