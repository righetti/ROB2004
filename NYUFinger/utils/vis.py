import time
import mujoco
import mujoco.viewer
import numpy as np
import os
from NYUFinger import ASSETS_PATH

class NYUFingerVisualizer:
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

    def show(self, q):
        q = np.array(q)
        assert q.shape==(3,), 'Wrong q shape! The shape should be: (3,)'
        self.data.qpos[:]=q
        mujoco.mj_step(self.model, self.data)
        if self.render:
            self.viewer.sync()

    def close(self):
        if self.render:
            self.viewer.close()
