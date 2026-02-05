import numpy as np
import zerorpc
import time
from dataclasses import dataclass
import threading
import matplotlib.pyplot as plt
from NYUFinger.real import NYUFingerHardware
import zmq
import msgpack # Efficient binary serialization (pip install msgpack)
from lerobot.policies.act.modeling_act import ACTPolicy
import pyrealsense2 as rs


class RealSenseCamera:
    """
    A class for interacting with a RealSense cameras.

    Args:
        callback_fn (callable, optional): The callback function to process frames.
        camera_serial_no (str, optional): The serial number of the camera.
        VGA (bool): Set to True for VGA resolution, False for HD resolution.
        color_fps (int): Frame rate for color stream (frames per second).
        depth_fps (int): Frame rate for depth stream (frames per second).
        enable_imu (bool): Enable or disable IMU stream.
        enable_depth (bool): Enable or disable depth stream.
        enable_color (bool): Enable or disable color stream.
        enable_ir (bool): Enable or disable infrared stream.
        emitter_enabled (bool): Enable or disable emitter for the depth sensor.
        align_to_color (bool): Align depth and IR streams to color.

    Attributes:
        callback_fn (callable, optional): The callback function to process frames.
        camera_serial_no (str): The serial number of the camera.
        VGA (bool): True if VGA resolution, False if HD resolution.
        color_fps (int): Frame rate for color stream (frames per second).
        depth_fps (int): Frame rate for depth stream (frames per second).
        enable_imu (bool): True if IMU stream is enabled.
        enable_depth (bool): True if depth stream is enabled.
        enable_color (bool): True if color stream is enabled.
        enable_ir (bool): True if infrared stream is enabled.
        emitter_enabled (bool): True if emitter is enabled for the depth sensor.
        align_to_color (bool): True if depth and IR streams are aligned to color.
    """
    def __init__(self, callback_fn = None, 
                       camera_serial_no=None, 
                       VGA = True,
                       color_fps=60,
                       depth_fps=90, 
                       enable_imu=False,
                       enable_depth=True, 
                       enable_color=True, 
                       enable_ir=True, 
                       emitter_enabled=True,
                       align_to_color = False):
        
        self.callback_fn = callback_fn
        self.camera_serial_no = camera_serial_no
        self.VGA = VGA
        self.color_fps = color_fps
        self.depth_fps = depth_fps
        self.enable_depth = enable_depth
        self.enable_color = enable_color
        self.align_to_color = align_to_color
        self.enable_ir = enable_ir
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.enable_imu = enable_imu
        if self.camera_serial_no is None:
            # Get device product line for setting a supporting resolution
            pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
            pipeline_profile = self.config.resolve(pipeline_wrapper)
            device = pipeline_profile.get_device()
            serial_no = str(device.get_info(rs.camera_info.serial_number))
            self.camera_serial_no = serial_no
        
        # Enable the streams for the connected device with the requested serial number
        print('Enabling streams for camera: ', self.camera_serial_no)
        self.config.enable_device(self.camera_serial_no)
        if VGA:
            img_size = (640, 480)
            if color_fps > 60:
                self.color_fps = 60
                print('Warning: VGA color fps cannot be higher than 60')
            if depth_fps > 90:
                self.depth_fps = 90
                print('Warning: VGA depth/infrared fps cannot be higher than 90')
        else:
            img_size = (1280, 720)
            if color_fps > 30:
                self.color_fps = 30
                print('Warning: HD color fps cannot be higher than 30')
            if depth_fps > 30:
                self.depth_fps = 30
                print('Warning: HD depth/infrared fps cannot be higher than 30')
        
        if enable_depth:
            self.config.enable_stream(rs.stream.depth, img_size[0], img_size[1], rs.format.z16, self.depth_fps)
        if enable_color:
            self.config.enable_stream(rs.stream.color, img_size[0], img_size[1], rs.format.bgr8, self.color_fps)
        if enable_ir:
            self.config.enable_stream(rs.stream.infrared, 1, img_size[0], img_size[1], rs.format.y8, self.depth_fps)
            self.config.enable_stream(rs.stream.infrared, 2, img_size[0], img_size[1], rs.format.y8, self.depth_fps)
        if self.enable_imu:
            self.config.enable_stream(rs.stream.accel)
            self.config.enable_stream(rs.stream.gyro)

        self.profile = self.pipeline.start(self.config)
        self.depth_sensor = self.profile.get_device().first_depth_sensor()
        if emitter_enabled:
            self.depth_sensor.set_option(rs.option.emitter_enabled, 1)
        else:
            self.depth_sensor.set_option(rs.option.emitter_enabled, 0)
        
        # Start the thread for grabbing frames
        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._run_grab_frames)
        self._thread.start()
        #Get stream profiles
        self.depth_profile = self.profile.get_stream(rs.stream.depth).as_video_stream_profile()
        self.color_profile = self.profile.get_stream(rs.stream.color).as_video_stream_profile()
        self.ir1_profile = self.profile.get_stream(rs.stream.infrared,1).as_video_stream_profile()
        self.ir2_profile = self.profile.get_stream(rs.stream.infrared,2).as_video_stream_profile()
        #Depth frame aligner
        self.align = rs.align(rs.stream.color)
        self.color_frame = None
        self.depth_frame = None

    def _run_grab_frames(self):
        """
        Private method to continuously grab frames in a separate thread.
        """
        while not self._stop_event.is_set():
            self.grab_frames()
            if self.callback_fn is not None:
                self.callback_fn(self.color_frame, self.depth_frame, self.ir1_frame, self.ir2_frame)

    def close(self):
        """
        Closes the RealSenseCamera object, stopping the frame grabbing thread and the pipeline.
        """
        # Stop the thread
        self._stop_event.set()
        self._thread.join()
        # Stop the pipeline
        self.pipeline.stop()

    def grab_frames(self):
        """
        Grabs frames from the RealSense camera and stores them in instance variables.
        """
        frames = self.pipeline.wait_for_frames()
        if self.align_to_color:
            frames = self.align.process(frames)
        if frames is None:
            print('Warning: failed to grab frames')
            self.close()
            
        if self.enable_depth:
            self.depth_frame = np.asanyarray(frames.get_depth_frame().get_data())
        else:
            self.depth_frame = None
        if self.enable_color:
            self.color_frame = np.asanyarray(frames.get_color_frame().get_data())
        else:
            self.color_frame = None
        if self.enable_ir:
            self.ir1_frame =   np.asanyarray(frames.get_infrared_frame(1).get_data())
            self.ir2_frame =   np.asanyarray(frames.get_infrared_frame(2).get_data())
        else:
            self.ir1_frame = None
            self.ir2_frame = None
        
        if self.enable_imu:
            self.accel_frame = frames.first_or_default(rs.stream.accel)
            self.gyro_frame = frames.first_or_default(rs.stream.gyro)
            # Optionally convert the IMU frames to arrays as needed, e.g., np.asanyarray(...)
        else:
            self.accel_frame = None
            self.gyro_frame = None

    def getIntrinsics(self):
        """
        Gets the intrinsic parameters of cameras.

        Returns:
            dict: A dictionary containing intrinsics for RGB, IR1, IR2, and Depth streams.
        """
        rgb_intr = self.color_profile.get_intrinsics()
        ir1_intr = self.ir1_profile.get_intrinsics()
        ir2_intr = self.ir2_profile.get_intrinsics()
        depth_intr = self.depth_profile.get_intrinsics()

        ir1_intr = self.parseIntr(ir1_intr)
        ir2_intr = self.parseIntr(ir2_intr)
        depth_intr = self.parseIntr(depth_intr)
        rgb_intr = self.parseIntr(rgb_intr)

        return {'RGB':rgb_intr,
                'IR1':ir1_intr,
                'IR2':ir2_intr,
                'Depth':depth_intr}
    
    def getExtrinsics(self):
        """
        Gets the extrinsics between different camera streams.

        Returns:
            dict: A dictionary containing pose of all streams with respect to depth/infrared1.
        """
        ir1_T_ir2 = self.ir2_profile.get_extrinsics_to(self.ir1_profile) # Pose of ir2 with respect to ir1
        ir1_T_rgb = self.color_profile.get_extrinsics_to(self.ir1_profile) # Pose of rgb with respect to ir1
        ir1_T_ir2 = self.toPose(ir1_T_ir2)
        ir1_T_rgb = self.toPose(ir1_T_rgb)
        return {'ir1_T_ir2':ir1_T_ir2,
                'ir1_T_rgb':ir1_T_rgb}

    def toPose(self,e):
        """
        Converts a RealSense extrinsics object to a 4x4 transformation matrix.

        Args:
            e (rs.extrinsics): The extrinsics object.

        Returns:
            numpy.ndarray: A 4x4 transformation matrix representing the extrinsics.
        """
        R = np.array(e.rotation).reshape(3,3)
        t = np.array(e.translation).reshape(3,1)
        return np.vstack([np.hstack([R,t]), np.array([0,0,0,1])])

    def parseIntr(self,intr):
        """
        Parses intrinsics data into a dictionary for easier access.

        Args:
            intr (rs.intrinsics): The intrinsics object.

        Returns:
            dict: A dictionary containing intrinsics data.
        """
        h,w = intr.height, intr.width
        fx, fy = intr.fx,intr.fy
        cx,cy = intr.ppx,intr.ppy
        dist = np.array(intr.coeffs)
        K = np.array([fx, 0, cx,
                    0,fy, cy,
                    0, 0, 1]).reshape(3,3)
        return {'size':(w,h),
                'fx':fx,
                'fy':fy,
                'cx':cx,
                'cy':cy,
                'K':K,
                'D':dist}   


class NYUFingerHardwareV2:
    def __init__(self, 
                 robot_ip='192.168.123.10',
                 dt=0.01):
        self.robot = zerorpc.Client()
        self.robot.connect(f"tcp://{robot_ip}:4242")
        self.q_raw = np.zeros(3)
        self.q_dir = np.array([-1, 1, -1])
        self.dt = dt
        self.running = True
        self.alpha = 0.9
        self.tau_alpha = 0.9
        self.tau_f = np.zeros(3) # filtered torque
        self.dq_f = np.zeros(3) # filtered velocity
        
        # Reset the relative encoder value
        state = self.robot.getJointStates()
        self.q0_rel = np.array([state[f'joint_{i+1}']['q'] for i in range(3)])
        # Get the current absolute joint angles from the absolute joint encoders
        self.q_offset = self.getAbsoluteJointAngles()

    def getAbsoluteJointAngles(self):
        abs_state = self.robot.getAbsJointStates()
        q_abs_offset = np.array([10.4, 0.88, 3.84])
        q_abs = np.array([s['q_abs'] for s in abs_state.values()]) - q_abs_offset
        return q_abs * np.array([0.5, 0.91, 1.]) * np.array([-1, -1, -1])
    
    def getAbsoluteJointAnglesRaw(self):
        abs_state = self.robot.getAbsJointStates()
        q_abs = np.array([s['q_abs'] for s in abs_state.values()])
        return q_abs

    def get_state(self):
        state = self.robot.getJointStates()
        q = np.array([state[f'joint_{i+1}']['q'] for i in range(3)])
        dq = np.array([state[f'joint_{i+1}']['dq'] for i in range(3)])
        tau = np.array([state[f'joint_{i+1}']['tau'] for i in range(3)])
        
        self.q_raw = q.copy()
        q = (q - self.q0_rel) * self.q_dir + self.q_offset
        dq = dq * self.q_dir
        
        # Filter velocity
        self.dq_f = self.alpha * self.dq_f + (1 - self.alpha) * dq
        self.tau_f = self.tau_alpha*self.tau_f + (1-self.tau_alpha)*tau
        
        # Return q, filtered velocity, and torque
        return q, self.dq_f, self.tau_f

    def send_joint_cmd(self, q, dq, joint_torques):
        tauff = joint_torques * self.q_dir
        qdes = (q - self.q_offset) * self.q_dir + self.q0_rel
        dqdes = dq * self.q_dir
        command = {f'joint_{i+1}': {'q': qdes[i], 'dq': dqdes[i], 'tau': tauff[i]} for i in range(3)}
        self.robot.setJointCommand(command)

    def reset_sensors(self, q0=np.zeros(3)):
        pass

    def get_latest_action(self):
        action_raw = self.robot.getJointCommand()
        q_raw = np.array([action_raw[f'joint_{i+1}']['q'] for i in range(3)])
        q_cmd = (q_raw - self.q0_rel) * self.q_dir + self.q_offset
        return q_cmd


# --- Setup ---
robot = NYUFingerHardwareV2()

from lerobot.datasets.lerobot_dataset import LeRobotDataset
dataset = LeRobotDataset(repo_id = "nyufinger_dataset", root = "/home/rooholla/projects/nyu-finger/ROB2004/data/cube-to-bag-relative", video_backend='pyav')
policy = ACTPolicy.from_pretrained("/home/rooholla/projects/lerobot/outputs/train/place_cubes-act-relative-no-state/checkpoints/last/pretrained_model")

from lerobot.policies.factory import make_pre_post_processors
preprocessor, postprocessor = make_pre_post_processors(
        policy_cfg=policy,
        pretrained_path="/home/rooholla/projects/lerobot/outputs/train/place_cubes-act-relative-no-state/checkpoints/last/pretrained_model",
        dataset_stats=dataset.meta.stats,
        # The inference device is automatically set to match the detected hardware, overriding any previous device settings from training to ensure compatibility.
        preprocessor_overrides={"device_processor": {"device": str(policy.config.device)}},
    )
policy = policy.eval().cuda()
import torch
import cv2

def make_policy_input(img, q_robot, dq_robot, tau_robot):
    img = torch.from_numpy(cv2.cvtColor(img, cv2.COLOR_BGR2RGB).astype(np.float32)/255.0).permute(2,0,1).unsqueeze(0)
    # img = torch.from_numpy(img.astype(np.float32)/255.0).permute(2,0,1).unsqueeze(0)
    
    input_features = {
        'observation.images.camera_0': img.cuda(),  
        'observation.state': torch.from_numpy(np.concatenate([q_robot, tau_robot], axis=0)).unsqueeze(0).to(torch.float32).cuda(),

    }
    return input_features
# try:
    # Indefinite loop, relies on KeyboardInterrupt to stop
camera = RealSenseCamera()
time.sleep(2.0)  # Allow camera to warm up
preprocessor.reset()
postprocessor.reset()
policy.eval()
a0 = robot.get_latest_action().copy()
while True:
    tic = time.time()
    # Get current states
    q_robot, dq_robot, tau_robot = robot.get_state()
    if camera.color_frame is not None:
        # img = cv2.cvtColor(camera.color_frame, cv2.COLOR_BGR2RGB)
        obs = make_policy_input(camera.color_frame, q_robot, dq_robot, tau_robot)
        cv2.imshow('camera', camera.color_frame)
        cv2.waitKey(1)
        obs_processed = preprocessor(obs)
        with torch.no_grad():
            action = policy.select_action(obs_processed)
            action = postprocessor(action).cpu().numpy()    
        # offset = torch.tensor([-0.0086, -0.0155,  0.0062]).view(1, 3).numpy()
        # action = action - offset
        a0 += action.squeeze()
        # action = robot.get_state()[0] + action.squeeze()
        # robot.send_joint_cmd(action, np.zeros(3), np.zeros(3))
        robot.send_joint_cmd(a0, np.zeros(3), np.zeros(3))
    while time.time()-tic < 0.05:
        time.sleep(0.0005)
