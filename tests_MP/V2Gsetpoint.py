import math
from datetime import time

import numpy as np


def quaternion_product(q0, q1):
    x0, y0, z0, w0 = q0
    x1, y1, z1, w1 = q1
    q0q1_w = w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1
    q0q1_x = w0 * x1 + x0 * w1 + y0 * z1 - z0 * y1
    q0q1_y = w0 * y1 - x0 * z1 + y0 * w1 + z0 * x1
    q0q1_z = w0 * z1 + x0 * y1 - y0 * x1 + z0 * w1
    return q0q1_x, q0q1_y, q0q1_z, q0q1_w


def yaw2quat(yaw):  # input yaw in degree
    yaw = yaw * math.pi / 180
    qx, qy = 0, 0
    qz = math.sin(yaw / 2)
    qw = math.cos(yaw / 2)
    return qx, qy, qz, qw


def quat2yaw(q):
    yaw = 2 * math.acos(q[3])
    return yaw * 180 / math.pi


setpoint = (0.2, -0.1, 0.5, -90)  # Vicon
setpoint = np.array((setpoint[0]*1000, setpoint[1]*1000, setpoint[2]*1000, setpoint[3]))

V2G_tr = np.array([-29.46296348,  48.21012761,  18.95172748, 0])#[0.5, 0.2, 0, 0])

# V2G_coord = (0, 0, -0.7071067811865475, 0.7071067811865476)
V2G_coord = [-0.01954052, +0.04656694,  -0.0458679,   0.9976702] # [0, 0, -0.258819, 0.9659258]
V2G_conj_coord = [0.01954052, -0.04656694,  0.0458679,   0.9976702] #[0, 0, 0.258819, 0.9659258]
# (0, 0, 0.7071067811865475, 0.7071067811865476)

V2B_tr = np.array([setpoint[0], setpoint[1], setpoint[2], 0])
V2B_tr_G = quaternion_product(V2G_coord,
                              quaternion_product(V2B_tr - V2G_tr,
                                                 V2G_conj_coord))
Btr_G = np.array([V2B_tr_G[0], V2B_tr_G[1], V2B_tr_G[2]])
print(Btr_G)

V2B_rot = yaw2quat(setpoint[3])
Brot_G = quaternion_product(V2G_coord, V2B_rot)
print(quat2yaw(Brot_G))
