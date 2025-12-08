import zmq
import msgpack
import time
import numpy as np
import pyrealsense2 as rs
from pynput import keyboard
from pathlib import Path
import torch
import threading
import cv2

DATASET_ROOT = "/home/rooholla/projects/nyu-finger/ROB2004/data"
REPO_ID = "nyufinger_dataset"
TASK = "cubes"

# --- ZMQ Subscriber (Same as before) ---
class RobotDataSubscriber:
    def __init__(self, endpoint="ipc:///tmp/robot_data.ipc"):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.set_hwm(1)
        self.socket.connect(endpoint)
        self.socket.setsockopt(zmq.SUBSCRIBE, b"")

    def receive(self, blocking=True):
        try:
            flags = 0 if blocking else zmq.NOBLOCK
            data = self.socket.recv(flags=flags)
            return msgpack.unpackb(data, raw=False)
        except zmq.Again:
            return None

    def close(self):
        self.socket.close()
        self.context.term()

# --- Camera (Same as before) ---
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

import os
import os
import shutil
import numpy as np
from pathlib import Path
from lerobot.datasets.lerobot_dataset import LeRobotDataset

class DataRecorder:
    def __init__(self, 
                 repo_id=REPO_ID, 
                 root=DATASET_ROOT, 
                 fps=50, 
                 robot_type="nyufinger"):
        
        self.repo_id = repo_id
        self.root = Path(root)
        self.dataset_path = self.root / self.repo_id
        self.fps = fps
        self.robot_type = robot_type

        # --- Dataset Initialization Logic ---
        # If dataset exists but is corrupted (missing meta), clean it up
        # LeRobot v2/v3 usually creates a 'meta' folder. 
        print(self.dataset_path.exists())
        print(self.dataset_path)
        if self.dataset_path.exists() and not (self.dataset_path / "meta").exists():
            print(f"[DataRecorder] Cleaning corrupted/empty dataset at {self.dataset_path}")
            shutil.rmtree(self.dataset_path)

        if self.dataset_path.exists():
            print(f"[DataRecorder] Loading existing dataset: {self.dataset_path}")
            self.dataset = LeRobotDataset(repo_id = self.repo_id, root = str(self.dataset_path))
        else:
            print(f"[DataRecorder] Creating new dataset: {self.dataset_path}")
            self.dataset = LeRobotDataset.create(
                repo_id=str(self.repo_id),
                fps=self.fps,
                root=str(self.dataset_path),
                robot_type=self.robot_type,
                features=self._generate_features(),
                use_videos=True,
                image_writer_threads=8  # Parallel MP4 encoding
            )

    def _generate_features(self):
        """Defines the feature schema for NYUFinger."""
        return {
            "observation.images.camera_0": {
                "dtype": "video",
                "shape": (480, 640, 3),
                "names": ["height", "width", "channel"],
            },
            "observation.state": {
                "dtype": "float32",
                "shape": (3,),
                "names": ["joint_1", "joint_2", "joint_3"],
            },
            "observation.velocity": {
                "dtype": "float32",
                "shape": (3,),
                "names": ["joint_1", "joint_2", "joint_3"],
            },
            "observation.effort": {
                "dtype": "float32",
                "shape": (3,),
                "names": ["joint_1", "joint_2", "joint_3"],
            },
            "action": {
                "dtype": "float32",
                "shape": (3,),
                "names": ["joint_1", "joint_2", "joint_3"],
            },
        }

    def add_state(self, robot_data, image, task="fingers-cubes"):
        """
        Adds a single frame to the buffer.
        
        Args:
            robot_data (dict): Must contain 'q', 'dq', 'tau', 'action' as numpy arrays.
            image (np.ndarray): HxWxC RGB image.
        """
        # Ensure data types match the schema
        frame = {
            "observation.images.camera_0": image,
            "observation.state": np.array(robot_data['q']).astype(np.float32),
            "observation.velocity": np.array(robot_data['dq']).astype(np.float32),
            "observation.effort": np.array(robot_data['tau']).astype(np.float32),
            "action": np.array(robot_data['action']).astype(np.float32),
            "task": task
        }
        
        self.dataset.add_frame(frame)

    def clear_buffer(self):
        """Discard current unsaved episode data."""
        self.dataset.clear_episode_buffer()

    def save_episode(self):
        """Commits the buffered frames to disk as an episode."""
        print("[DataRecorder] Saving episode...")
        self.dataset.save_episode()
        print(f"[DataRecorder] Episode saved. Total episodes: {self.dataset.num_episodes}")

    def finalize(self):
        """Calculate statistics and finalize dataset structure."""
        print("[DataRecorder] Consolidating dataset (calculating stats)...")
        self.dataset.consolidate()


# --- Global State ---
is_recording = False
should_quit = False
episode_buffer = []

def on_press(key):
    global is_recording, should_quit, episode_buffer
    try:
        if key.char == 'r':
            if not is_recording:
                print("\n[RECORDING STARTED]")
                is_recording = True
        elif key.char == 's':
            if is_recording:
                print("\n[RECORDING STOPPED]")
                is_recording = False
        elif key.char == 'q':
            print("\n[QUITTING]")
            should_quit = True
            return False
    except AttributeError:
        pass

if __name__ == "__main__":
    subscriber = RobotDataSubscriber()
    camera = RealSenseCamera()
    recorder = DataRecorder(fps=50)
    listener = keyboard.Listener(on_press=on_press)
    listener.start()
    
    print("\n=== LEROBOT RECORDER READY ===")
    print(" [r] Start  |  [s] Stop & Save  |  [q] Quit")
    
    last_save_check = False
    
    try:
        while not should_quit:
            # Sync with robot (50Hz)
            # Note: If robot is 50Hz and Dataset is 30Hz, you might want to decimate here
            # or just record at 50Hz (set FPS=50 above)
            robot_data = subscriber.receive(blocking=True)
            
            if robot_data is None:
                continue

            frame = camera.color_frame # RGB numpy array

            if is_recording and frame is not None:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                recorder.add_state(robot_data, frame.copy(), task=TASK)
            
            if last_save_check and not is_recording:
                recorder.save_episode()
                recorder.clear_buffer()
            
            last_save_check = is_recording

    except KeyboardInterrupt:
        pass
    finally:
        print("Finalizing dataset...")
        # dataset.consolidate() # Important: writes metadata and stats
        subscriber.close()
        camera.close()
        listener.stop()
        recorder.dataset.finalize()
        print("Done.")
