import time
import zerorpc
import numpy as np
from NYUFinger.utils.vis import NYUFingerVisualizer
vis = NYUFingerVisualizer()

class visRPC:
    def show(self, q):
        vis.show(np.array(q))

s = zerorpc.Server(visRPC())
s.bind("tcp://0.0.0.0:4242")
s.run()