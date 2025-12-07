import zmq
import msgpack
import time
import numpy as np
import h5py
import pyrealsense2 as rs
import cv2
from datetime import datetime
from pynput import keyboard
import threading
import os
from tqdm import tqdm
from concurrent.futures import ThreadPoolExecutor
# Set numpy print options
np.set_printoptions(precision=2, suppress=True)

class RobotDataSubscriber:
    def __init__(self, endpoint="ipc:///tmp/robot_data.ipc"):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        print(f"Connecting to subscriber endpoint: {endpoint}")
        self.socket.connect(endpoint)
        self.socket.setsockopt(zmq.SUBSCRIBE, b"")

    def receive(self, blocking=True):
        try:
            flags = 0 if blocking else zmq.NOBLOCK
            data = self.socket.recv(flags=flags)
            payload = msgpack.unpackb(data, raw=False)
            return payload
        except zmq.Again:
            return None
        except Exception as e:
            print(f"Error receiving data: {e}")
            return None

    def close(self):
        self.socket.close()
        self.context.term()

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

def compress_image(img):
    """Helper function for threaded compression"""
    success, encoded_img = cv2.imencode('.jpg', img, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
    if success:
        return encoded_img.flatten()
    else:
        return np.array([], dtype='uint8')

def save_episode_to_hdf5(buffer, output_dir="data"):
    if not buffer:
        print("Buffer empty, nothing to save.")
        return

    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = os.path.join(output_dir, f"episode_{timestamp}.h5")
    
    print(f"Processing {len(buffer)} frames...")
    
    # Unpack buffer
    robot_data_list, images_list = zip(*buffer)
    
    # 1. Prepare Robot Data (Standard numpy arrays)
    q_pos = np.array([d['q'] for d in robot_data_list])
    q_vel = np.array([d['dq'] for d in robot_data_list])
    effort = np.array([d['tau'] for d in robot_data_list])
    action = np.array([d['action'] for d in robot_data_list])
    timestamps = np.array([d['timestamp'] for d in robot_data_list])
    
    # 2. Parallel JPEG Compression
    # Using ThreadPoolExecutor to utilize multiple cores for compression
    print("Compressing images...")
    jpeg_images = []
    with ThreadPoolExecutor() as executor:
        # map preserves order, tqdm shows progress
        jpeg_images = list(tqdm(executor.map(compress_image, images_list), total=len(images_list), unit="img"))

    # 3. Save to HDF5
    print(f"Saving to {filename}...")
    with h5py.File(filename, 'w') as f:
        obs = f.create_group('observations')
        
        # Create variable-length uint8 datatype
        dt = h5py.vlen_dtype(np.dtype('uint8'))
        
        # Save JPEGs
        dset = obs.create_dataset('images', (len(jpeg_images),), dtype=dt)
        for i, data in enumerate(jpeg_images):
            dset[i] = data
            
        dset.attrs['format'] = 'jpeg'
        
        # Save Robot State
        obs.create_dataset('qpos', data=q_pos)
        obs.create_dataset('qvel', data=q_vel)
        obs.create_dataset('effort', data=effort)
        f.create_dataset('action', data=action)
        f.create_dataset('timestamp', data=timestamps)
        
        f.attrs['num_samples'] = len(buffer)

    print(f"Save complete! File size: {os.path.getsize(filename) / (1024*1024):.2f} MB")


# --- Global Flags for Keyboard Control ---
is_recording = False
should_quit = False
episode_buffer = []

def on_press(key):
    global is_recording, should_quit, episode_buffer
    try:
        if key.char == 'r':
            if not is_recording:
                print("\n[RECORDING STARTED]")
                episode_buffer = [] # Clear buffer
                is_recording = True
        elif key.char == 's':
            if is_recording:
                print("\n[RECORDING STOPPED]")
                is_recording = False
                # Save is handled in the main loop to avoid blocking the listener
        elif key.char == 'q':
            print("\n[QUITTING]")
            should_quit = True
            return False # Stop listener
    except AttributeError:
        # Handle special keys if needed (e.g. Esc)
        if key == keyboard.Key.esc:
            should_quit = True
            return False

# --- Main Execution ---
if __name__ == "__main__":
    # 1. Initialize Sensors
    subscriber = RobotDataSubscriber()
    camera = RealSenseCamera()
    
    # 2. Start Keyboard Listener (Non-blocking)
    listener = keyboard.Listener(on_press=on_press)
    listener.start()
    
    print("\n" + "="*40)
    print("DATA RECORDER READY")
    print("Commands:")
    print("  [r] : Start Recording Episode")
    print("  [s] : Stop Recording & Save")
    print("  [q] : Quit")
    print("="*40 + "\n")

    last_save_check = False # To track state changes

    try:
        while not should_quit:
            # 1. Get Robot Data (Blocking syncs loop to robot freq ~50Hz)
            robot_data = subscriber.receive(blocking=True)
            
            if robot_data is None:
                continue

            # 2. Get Camera Frame (Non-blocking check inside wait_for_frames usually fast enough)
            # Note: If robot is 50Hz and Camera is 30Hz, we might get duplicate frames 
            # or need to align timestamps. Here we just grab the latest available.
            if camera.color_frame is not None:
                frame = camera.color_frame.copy()

            # 3. Recording Logic
            if is_recording:
                if frame is not None:
                    # Store tuple of (data, image)
                    episode_buffer.append((robot_data, frame))
                    
                    # Visual feedback
                    if len(episode_buffer) % 10 == 0:
                        print(f"\rRecording... {len(episode_buffer)} frames", end="")
            
            # 4. Save Trigger (Detect falling edge of is_recording)
            if last_save_check and not is_recording:
                # User just pressed 's'
                print() # New line
                if len(episode_buffer) > 0:
                    save_episode_to_hdf5(episode_buffer)
                    episode_buffer = []
                else:
                    print("Empty episode, discarded.")
            
            last_save_check = is_recording

            # Optional: Show camera feed (Slows down loop slightly)
            # if frame is not None:
            #     cv2.imshow('Recorder Feed', frame)
            #     if cv2.waitKey(1) & 0xFF == ord('q'):
            #         break

    except KeyboardInterrupt:
        print("\nForce stopping...")

    finally:
        subscriber.close()
        camera.close()
        listener.stop()
        cv2.destroyAllWindows()
        print("Clean exit.")
