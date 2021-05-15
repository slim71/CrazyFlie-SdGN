# sequenza in terna globale

import logging
import os
import time
from datetime import datetime
from pathlib import Path
from own_module import crazyfun as crazy
import numpy as np
# from test_SV import sequences as seq
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

from own_module.log_estimation_manager import LogEstimationManager
from own_module.vicon_manager_old import ViconManager

URI = 'radio://0/80/2M'
DEFAULT_HEIGHT = 0.4

position_estimate = [0, 0, 0]
attitude_estimate = [0, 0, 0]

# -----------------------------------------------------------------------------
# ----------------------------------DEBUG--------------------------------------
# -----------------------------------------------------------------------------
filename = "../other_or_old/Logs/LogFile_CrazyFlie_Test2_" \
           + datetime.now().strftime("%Y%m%d_%H%M%S")

logname = filename + ".log"
filename = filename + ".txt"
logging.basicConfig(filename=logname,
                    level=logging.DEBUG,
                    filemode='a',
                    format="%(asctime)s [%(levelname)s]: %(message)s")

# -----------------------------------------------------------------------------
# sequence in global Crazy frame
sequence = [
    (0, 0.3, DEFAULT_HEIGHT, 0),
    (0.2, 0, DEFAULT_HEIGHT, 0),
    (-0.2, 0, DEFAULT_HEIGHT, 0),
    (0, 0, 0.3, 0)
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

            with open(ff, 'a') as descr:
                print("% x, y, z, qx, qy, qz, qw", file=descr)
                for setpoint in sequence:
                    for i in range(20):
                        pos_drone, quat_drone = vicon.get_drone_pose()
                        cf.extpos.send_extpose(pos_drone[0], pos_drone[1], pos_drone[2],
                                               quat_drone[0], quat_drone[1], quat_drone[2], quat_drone[3])
                        cf.commander.send_position_setpoint(setpoint[0], setpoint[1], setpoint[2], setpoint[3])
                        time.sleep(0.1)

            cf.commander.send_stop_setpoint()
