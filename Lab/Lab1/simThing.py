import numpy as np
import matplotlib.pyplot as plt
import time
np.set_printoptions(precision=3) #Limit the print precision to 3 digits
np.set_printoptions(suppress=True) # Disable scientifi number notation when printing numpy arrays

from NYUFinger.sim import NYUFingerSimulator
robot_sim = NYUFingerSimulator()

print('g')
q, dq = robot_sim.get_state()
print('Joint Position:')
print(q)
print('Joint Velocity:')
print(dq)