# sequenza in terna vicon convertita poi in globale

import logging
import time
import os
from pathlib import Path
from datetime import datetime
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from own_module.log_estimation_manager import LogEstimationManager
from own_module.vicon_manager import ViconManager
from own_module import crazyfun as crazy
import math
from test_SV import sequences as seq
from own_module import converter


URI = 'radio://0/80/2M'
DEFAULT_HEIGHT = 0.5
RADIUS = 0.5

position_estimate = [0, 0, 0]
attitude_estimate = [0, 0, 0]

# -----------------------------------------------------------------------------
# ----------------------------------DEBUG--------------------------------------
# -----------------------------------------------------------------------------
filename = "../other_or_old/Logs/LogFile_CrazyFlie_Test3_" \
           + datetime.now().strftime("%Y%m%d_%H%M%S")

logname = filename + ".log"
filename = filename + ".txt"
logging.basicConfig(filename=logname,
                    level=logging.DEBUG,
                    filemode='a',
                    format="%(asctime)s [%(levelname)s]: %(message)s")

# -----------------------------------------------------------------------------
# sequence in vicon frame [in mm!!]
# sequence_V = [(0, 0, DEFAULT_HEIGHT, 0)]
# theta = 0.0
# for j in range(16):
# 	theta = theta + 2 * math.pi / 16
# 	sequence_V.append((RADIUS * math.cos(theta), RADIUS * math.sin(theta), DEFAULT_HEIGHT,
# 					   (theta+math.pi)*180/math.pi))
# sequence_V.append((0, 0, 0.3, math.pi*180/math.pi))
sequence_V = [
    (-383.92827523, 141.19788576, 512.13045348),
    (83.82958245, 470.39863495, 559.02640051),
    (-613.64755051, 404.607765, 554.85091315),
    (-493.06587464, -268.20454026, 553.15361612),
    (221.18592057, -157.22488258, 518.6648357),
    (95.557363, 518.20317237, 526.26778606),
    (-264.04658154, 118.62626847, 533.59377608)
]

if __name__ == '__main__':
    data_file = "./data_logs/" + datetime.now().strftime("__%Y%m%d_%H%M")
    data_file = data_file + ".txt"
    ff = os.path.normpath(os.path.join(Path(__file__).parent.absolute(),
                                       data_file))

    with LogEstimationManager() as log_est:
        with SyncCrazyflie(URI) as scf:
            cf = scf.cf
            log_est.simple_log_async(cf)
            log_est.reset_estimator(cf)
            vicon = ViconManager()
            time.sleep(0.5)
            V2G_dist, V2G_rot = vicon.get_global_frame()
            print("V2G_dist: ", V2G_dist)
            print("V2G_rot: ", V2G_rot)

            crazy.matlab_print("% setpoint_V setpoint_G")

            for setpoint_V in sequence_V:
                Btransl_G, Brot_G = converter.change2G_coord(
                    (setpoint_V[0], setpoint_V[1], setpoint_V[2]),
                    converter.yaw2quat(0),
                    V2G_dist, V2G_rot)
                setpoint_G = (Btransl_G[0], Btransl_G[1], Btransl_G[2], 0)
                print(setpoint_G)

                crazy.matlab_print(setpoint_V, setpoint_G)

            with open(ff, 'a') as descr:
                print("% x, y, z, qx, qy, qz, qw", file=descr)
                for s in setpoint_G:
                    logging.debug("new setpoint: %s", str(setpoint_V))

                    for i in range(20):
                        # cf.commander.send_position_setpoint(s[0], s[1], s[2], s[3])

                        pos_drone, quat_drone = vicon.get_drone_pose(V2G_dist, V2G_rot)
                        # print("Btransl_G: ", str(pos_drone), " Brot_G: ",
                        #       str(quat_drone))
                        print(pos_drone[0], pos_drone[1], pos_drone[2],
                              quat_drone[0], quat_drone[1], quat_drone[2],
                              quat_drone[3],
                              file=descr)
                        pos_est, att_est = log_est.get_estimation()
                        # print("pos: ", str(pos_est), "att: ", str(att_est))
                        print(pos_est[0], pos_est[1], pos_est[2],
                              att_est[0], att_est[1], att_est[2],
                              file=descr)
                        cf.extpos.send_extpose(pos_drone[0], pos_drone[1], pos_drone[2],
                                               quat_drone[0], quat_drone[1], quat_drone[2], quat_drone[3])
                        time.sleep(0.1)
                    time.sleep(1)

            cf.commander.send_stop_setpoint()
