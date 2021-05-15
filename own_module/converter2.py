import numpy as np


def quat_prod(q0, q1):  # both Vicon and CF use the form (x, y, z, w), NOT (w, x, y, z)
    x0, y0, z0, w0 = q0
    x1, y1, z1, w1 = q1

    q0q1_w = w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1
    q0q1_x = w0 * x1 + x0 * w1 + y0 * z1 - z0 * y1
    q0q1_y = w0 * y1 - x0 * z1 + y0 * w1 + z0 * x1
    q0q1_z = w0 * z1 + x0 * y1 - y0 * x1 + z0 * w1

    return q0q1_x, q0q1_y, q0q1_z, q0q1_w


def change2G_coord(V2B_transl, V2B_rot, V2G_rot):
    # transpose the rotation (conjugate the quaternion) to have a coordinate transformation from V to G
    V2G_coord = (-V2G_rot[0], -V2G_rot[1], -V2G_rot[2], V2G_rot[3])
    V2G_coord_conj = V2G_rot

    V2B_transl = np.array((V2B_transl[0], V2B_transl[1], V2B_transl[2], 0))  # position into quaternion
    V2B_transl_G = quat_prod(V2G_coord, quat_prod(V2B_transl, V2G_coord_conj))
    # V2B_transl_G = (V2B_transl_G[0]/1000, V2B_transl_G[1]/1000, V2B_transl_G[2]/1000)

    Brot_G = quat_prod(V2G_coord, V2B_rot)  # Body rotation in Global coordinates
    # Brot_G = (Brot_G[0], Brot_G[1], Brot_G[2], Brot_G[3])
    return V2B_transl_G, Brot_G