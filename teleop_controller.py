import numpy as np
import zerorpc
import time
from dataclasses import dataclass
import threading
import matplotlib.pyplot as plt
from NYUFinger.real import NYUFingerHardware
import zmq
import msgpack # Efficient binary serialization (pip install msgpack)

class DataStreamer:
    def __init__(self, endpoint="ipc:///tmp/robot_data.ipc"):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        # IPC uses a file path, avoiding "hardcoded ports"
        self.socket.bind(endpoint) 
        self.running = True
        self.queue = []
        self.lock = threading.Lock()
        
        # Start background thread
        self.thread = threading.Thread(target=self._stream_loop, daemon=True)
        self.thread.start()

    def publish(self, state_q, state_dq, tau, action):
        """
        Non-blocking publish. Pushes data to a local buffer.
        """
        payload = {
            "timestamp": time.time(),
            "q": state_q.tolist(),
            "dq": state_dq.tolist(),
            "tau": tau.tolist(),
            "action": action.tolist()
        }
        with self.lock:
            self.queue.append(payload)

    def _stream_loop(self):
        while self.running:
            # Send all pending messages
            with self.lock:
                if self.queue:
                    # Batch send or single send depending on preference
                    # Here we send one by one to preserve order
                    for msg in self.queue:
                        # msgpack is faster/smaller than json
                        self.socket.send(msgpack.packb(msg)) 
                    self.queue.clear()
            time.sleep(0.001) # Prevent CPU hogging

    def close(self):
        self.running = False
        self.thread.join()
        self.socket.close()
        self.context.term()


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


# --- Setup ---
robot = NYUFingerHardwareV2()
leader_robot = NYUFingerHardware(robot_ip='192.168.124.10', local_port=5001)

# --- Calibration Phase ---
input('Put the leader in home pose and press enter to continue ...')
leader_robot.reset_sensors()
input("Bring the leader robot to a close pose to the robot and press enter to continue...")

# --- Interpolation Phase ---
print("Starting smooth interpolation to leader position...")
interpolation_duration = 2.0  # seconds
interp_start_time = time.time()
dt = 0.02

# Capture starting states
start_q_robot, _, _ = robot.get_state()
target_q_leader, _ = leader_robot.get_state() 

while True:
    t = time.time() - interp_start_time
    if t > interpolation_duration:
        break
        
    alpha = np.clip(t / interpolation_duration, 0, 1)
    
    # Linear Interpolation (LERP)
    q_cmd = (1 - alpha) * start_q_robot + alpha * target_q_leader
    
    # Send interpolated command with zero velocity/torque
    robot.send_joint_cmd(q_cmd, np.zeros(3), np.zeros(3))
    time.sleep(dt)

print("Interpolation complete. Starting teleoperation... Press Ctrl+C to stop.")

# --- Main Teleoperation Loop ---
streamer = DataStreamer(endpoint="ipc:///tmp/robot_data.ipc")
SAFETY_THRESHOLD = 0.4  # Radians

try:
    # Indefinite loop, relies on KeyboardInterrupt to stop
    while True:
        tic = time.time()
        # Get current states
        q_leader, dq_leader = leader_robot.get_state()
        q_robot, dq_robot, tau_robot = robot.get_state()
        
        # 1. Calculate Error
        tracking_error = np.linalg.norm(q_leader - q_robot)
        
        # 2. Safety Check
        if tracking_error > SAFETY_THRESHOLD:
            print(f"[SAFETY STOP] Tracking error {tracking_error:.3f} exceeded limit {SAFETY_THRESHOLD}")
            break
            
        # 3. Send Command
        # Mirroring leader position
        robot.send_joint_cmd(q_leader, np.zeros(3), np.zeros(3))
        
        # 4. Stream Data
        streamer.publish(q_robot, dq_robot, tau_robot, q_leader)
        
        while time.time()-tic < 0.02:
            time.sleep(0.0005)

except KeyboardInterrupt:
    print("\nStopped by user.")

except Exception as e:
    print(f"\nAn error occurred: {e}")

finally:
    # Always ensure zero torque is sent when exiting
    print("Disabling robot control...")
    try:
        # Attempt to get state one last time to ensure safe shutdown
        current_q, _, _ = robot.get_state()
        robot.send_joint_cmd(current_q, np.zeros(3), np.zeros(3))
    except:
        # Fallback if robot connection is broken
        pass
        
    streamer.close()
