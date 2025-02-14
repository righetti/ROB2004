import time
import mujoco
import mujoco.viewer
import numpy as np
import os
from NYUFinger import ASSETS_PATH
import threading

class NYUFingerSimulator:
    def __init__(self, 
                 render=True, 
                 dt=0.001, 
                 xml_path=None,
                 ):

        if xml_path is None:
            self.model = mujoco.MjModel.from_xml_path(
                os.path.join(ASSETS_PATH, 'single_finger_scene.xml')
            )
        else:
            self.model = mujoco.MjModel.from_xml_path(xml_path)

        self.simulated = True
        self.data = mujoco.MjData(self.model)
        self.dt = dt
        _render_dt = 1 / 60
        self.render_ds_ratio = max(1, _render_dt // dt)

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
        self.renderer = None
        self.render = render
        self.step_counter = 0

        self.q0 = np.zeros(3)
        # self.reset()
        mujoco.mj_step(self.model, self.data)
        if self.render:
            self.viewer.sync()
        self.nv = self.model.nv
        self.tau_ff = np.zeros(3)
        self.latest_command_stamp = time.time()
        self.reset()
        self.running = True
        self.sim_thread = threading.Thread(target=self.sim_loop)
        self.sim_thread.start()

    def sim_loop(self):
        while self.running:
            # Reset the torque commands if the last command sent by the user is too old (older than 0.1 seconds)
            if time.time()-self.latest_command_stamp > 0.1: 
                self.tau_ff[:] = 0.
            self.step()
            time.sleep(self.dt)

    def reset(self, q0=None):
        if q0 is not None:
            q0 = np.array(q0)
            assert q0.shape==(3,), 'Wrong q0 shape! The shape should be (3,)'
            self.data.qpos[:]=q0
        else:
            self.data.qpos[2] = 0.8
        mujoco.mj_step(self.model, self.data)
        if self.render:
            self.viewer.sync()

    def get_state(self):
        return self.data.qpos[:3], self.data.qvel[:3]

    def send_joint_torque(self, torque):
        assert np.array(torque).shape == (3,), 'Wrong torque shape! The torque commnand should be a numpy array with shape (3,)'
        self.latest_command_stamp = time.time()
        self.tau_ff[:] = np.array(torque)

    def step(self):
        self.data.ctrl[:] = self.tau_ff
        self.step_counter += 1
        mujoco.mj_step(self.model, self.data)
        # Render every render_ds_ratio steps (60Hz GUI update)
        if self.render and (self.step_counter % self.render_ds_ratio) == 0:
            self.viewer.sync()

    def close(self):
        if self.render:
            self.viewer.close()
        self.running = False
        self.sim_thread.join()
