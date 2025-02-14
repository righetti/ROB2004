import socket
import struct
import numpy as np
import time
import threading
from dataclasses import dataclass

@dataclass
class RobotConfig:
    robot_ip: str
    robot_port: int
    local_port: int
    dds_freq: float
    DoF: int
    robot_name: str
    state_fields: list
    command_fields: list
    current2Torque: float
    gear_ratio: float
    max_torque: float

class TeensyUDPBridge():
    def __init__(self, 
                 cfg, 
                 user_callback=None):
        self.bridge_ip = cfg.robot_ip
        self.bridge_port = cfg.robot_port
        self.socket=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
        self.RX_RUNNING = True
        self.socket.bind(('0.0.0.0', cfg.local_port))
        self.rx_thread=threading.Thread(target=self.receivingThread)
        self.state = None
        self.latest_state_stamp = time.time()
        self.packet = None
        self.user_callback = user_callback
        self.rx_thread.start()

    def sendCommand(self, cmd):
        msg_format=f'{len(cmd)}f'
        payload = cmd
        arguments = [msg_format] + payload
        packet=struct.pack(*arguments)
        self.socket.sendto(packet, (self.bridge_ip, self.bridge_port))

    def getLatestState(self):
        return self.latest_state_stamp, self.state

    def receivingThread(self):
        while self.RX_RUNNING:
            packet = self.socket.recvmsg(4096)[0]
            state_msg_format=f'{len(packet)//4}f'
            data = struct.unpack(state_msg_format, packet)
            self.state = data
            self.latest_state_stamp = time.time()
            # Notify user if a callback is provided
            if self.user_callback is not None:
                print('Calling user callback')
                self.user_callback()

    def close(self):
        self.RX_RUNNING = False
        self.socket.close()
        self.rx_thread.join()

def get_last_msg(reader, topic_type):
    """ """
    last_msg = reader.take()

    if last_msg:
        while True:
            a = reader.take()
            if not a:
                break
            else:
                last_msg = a
    if last_msg:
        msg = last_msg[0]
        if type(msg) == topic_type:
            return msg
        else:
            return None

    else:
        return None
            
class NYUFingerHardware:
    def __init__(self):
        self.config = RobotConfig
        self.config.DoF = 3
        self.config.robot_name = 'finger1'
        self.config.current2Torque = 1.0
        self.config.gear_ratio = 9
        self.config.max_torque = 20.0
        self.config.robot_ip  = '192.168.123.10'
        self.config.robot_port = 5000
        self.config.local_port = 5000
        self.udp_bridge = TeensyUDPBridge(self.config)
        self.q_offset = np.zeros(3)
        self.q_raw = np.zeros(3)
        self.q_dir = np.array([-1, -1, -1])
    
    def get_state(self):
        stamp, teensy_state = self.udp_bridge.getLatestState()
        if teensy_state is not None:
            state = np.array(teensy_state)
            q = (state[:3]/self.config.gear_ratio)*self.q_dir
            dq = (state[3:6]/self.config.gear_ratio)*self.q_dir
            tau = (state[6:]*self.config.current2Torque * self.config.gear_ratio).tolist()
            self.q_raw = q.copy()
            self.dq_raw = dq.copy()
            return q-self.q_offset, dq
        else:
            return None, None
    
    def send_joint_torque(self, joint_torques):
        assert np.array(joint_torques).shape == (3,), 'Wrong torque shape! The torque commnand should be a numpy array with shape (3,)'
        udp_cmd = np.zeros(15)
        tau_ff = np.clip(joint_torques, -self.config.max_torque, self.config.max_torque)
        current = (np.array(tau_ff)/self.config.gear_ratio)/self.config.current2Torque
        udp_cmd[:3] = current*self.q_dir
        self.udp_bridge.sendCommand(udp_cmd.tolist())

    def reset_sensors(self, q0=np.zeros(3)):
        assert q0.shape==(3,), 'Wrong q0 shape! The shape should be (3,)'  
        for i in range(100):
            q, dq = self.get_state()
            time.sleep(0.01)
        if q is None:
            raise Exception('Could not reset read the states from the robot! Make sure the robot is powered on and connected.')
        # q = q_raw - q_offset -> q_offset = q_raw-q0
        self.q_offset[:] = self.q_raw - q0
        print(f'Successfully reset the sensor values to: {q0}')

