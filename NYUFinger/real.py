import numpy as np
import zerorpc
import time
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

def getQDQ(robot):
    state = robot.getJointStates()
    q = np.array([state[f'joint_{i+1}']['q'] for i in range(3)])
    dq = np.array([state[f'joint_{i+1}']['dq'] for i in range(3)])
    return q, dq

def setCommand(robot, q, dq, tau):
    command = {f'joint_{i+1}': {'q': q[i], 'dq': dq[i], 'tau': tau[i]} for i in range(3)}
    robot.setJointCommand(command)


class NYUFingerHardware:
    def __init__(self, 
                 robot_ip = '192.168.123.10', local_port = 5000):
        self.config = RobotConfig
        self.config.DoF = 3
        self.config.robot_name = 'finger1'
        self.config.current2Torque = 1.0
        self.config.gear_ratio = 9
        self.config.max_torque = 20.0
        self.config.robot_ip  = robot_ip
        self.config.robot_port = local_port
        self.config.local_port = local_port
        self.robot = zerorpc.Client()
        self.robot.connect(f"tcp://{robot_ip}:4242")
        self.q_offset = np.zeros(3)
        self.q_raw = np.zeros(3)
        self.q_dir = np.array([1, 1, 1])
    
    def get_state(self):
        try:
            state = self.robot.getJointStates()
            q = np.array([state[f'joint_{i+1}']['q'] for i in range(3)])
            dq = np.array([state[f'joint_{i+1}']['dq'] for i in range(3)])
            self.q_raw = q.copy()
            q = (q - self.q_offset)*self.q_dir
            dq = dq*self.q_dir
            return q, dq
        except:
            return None, None
    
    def send_joint_torque(self, joint_torques, q= np.zeros(3), dq=np.zeros(3)):
        assert np.array(joint_torques).shape == (3,), 'Wrong torque shape! The torque commnand should be a numpy array with shape (3,)'
        command = {f'joint_{i+1}': {'q': q[i], 'dq': dq[i], 'tau': joint_torques[i]} for i in range(3)}
        self.robot.setJointCommand(command)

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