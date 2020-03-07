import numpy as np
import math

def quaternion_multiply(quaternion1, quaternion0):
    w0, x0, y0, z0 = quaternion0
    w1, x1, y1, z1 = quaternion1
    return np.array([-x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0,
                     x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
                     -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
                     x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0], dtype=np.float64)

#vector
v = np.array([0,0,0,0.25])
#quaternion
Q = np.array([0.5, -0.5, 0.5, -0.5])
Qprime = np.array([Q[0], -Q[1], -Q[2], -Q[3]])
#want to get v = [0,0,-0.25]

print(quaternion_multiply(quaternion_multiply(Q,v), Qprime))
