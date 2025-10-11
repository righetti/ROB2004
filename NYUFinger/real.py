import numpy as np
import zerorpc

class NYUFingerHardware:
    def __init__(self, 
                 robot_ip = '192.168.123.10',
                 q_offset = np.array([10.69293916,   2.39383921,  3.78163332]),
                 dt=0.01):
        self.robot = zerorpc.Client()
        self.robot.connect(f"tcp://{robot_ip}:4242")
        self.q_offset = q_offset
        self.q_raw = np.zeros(3)
        self.q_dir = np.array([-1, 1, -1])
        self.dt = dt
        self.running = True
        # The control starts after the start of the main program
        self.Kp = 0.05 # 2.
        self.Kd = 0. # 0.04
        self.qdes = np.zeros(3) # reference position from MPC
        self.dqdes = np.zeros(3) # reference velocity from MPC
        self.tauff = np.zeros(3) # feed-forward torque from MPC
        self.alpha = 0.9
        self.dq_f = np.zeros(3) # filtered velocity
           
    def set_PD_loop_ref(self, active, ref):
        # called by MPC
        self.qdes = np.array(ref[:3])
        self.dqdes = np.array(ref[3:6])
        self.tauff = np.array(ref[6:]) 
        self.send_joint_cmd(self.qdes, self.dqdes, self.tauff)

    def get_state(self):
        state = self.robot.getJointStates()
        q = np.array([state[f'joint_{i+1}']['q'] for i in range(3)])
        dq = np.array([state[f'joint_{i+1}']['dq'] for i in range(3)])
        self.q_raw = q.copy()
        q = (q - self.q_offset)*self.q_dir
        dq = dq*self.q_dir
        self.dq_f = self.alpha*self.dq_f + (1-self.alpha)*dq
        return q, dq, self.dq_f # return joints position and filtered velocity

    def send_joint_cmd(self, q, dq, joint_torques):
        tauff = joint_torques*self.q_dir
        qdes = q*self.q_dir + self.q_offset
        dqdes = dq*self.q_dir
        command = {f'joint_{i+1}': {'q': qdes[i], 'dq': dqdes[i], 'tau': tauff[i]} for i in range(3)}
        self.robot.setJointCommand(command)

    def reset_sensors(self, q0=np.zeros(3)):
        pass