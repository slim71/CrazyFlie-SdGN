# Uso Commander anziché MC; classe vicon invia posizione Drone

import logging
import time
from datetime import datetime
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from own_module.log_estimation_manager import LogEstimationManager
from own_module.vicon_manager import ViconManager

URI = 'radio://0/80/2M'
DEFAULT_HEIGHT = 0.5
DESIRED_X = 0.4

position_estimate = [0, 0, 0]
attitude_estimate = [0, 0, 0]

# -----------------------------------------------------------------------------
# ----------------------------------DEBUG--------------------------------------
# -----------------------------------------------------------------------------
filename = "../other_or_old/Logs/LogFile_CrazyFlie_NOFLAG" \
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
    (0, 0, DEFAULT_HEIGHT, 0),
    (0.5, 0, DEFAULT_HEIGHT, 0),
    (-0.5, 0, DEFAULT_HEIGHT, 0),
    (0, 0, 0, 0),
]

if __name__ == '__main__':

    with LogEstimationManager() as log_est:
        with SyncCrazyflie(URI) as scf:
            cf = scf.cf
            log_est.simple_log_async(cf)
            log_est.reset_estimator(cf)
            vicon = ViconManager()
            for setpoint in sequence:
                for i in range(10):
                    pos_dr_vic = vicon.get_drone_pos()
                    cf.extpos.send_extpos(pos_dr_vic[0],
                                          pos_dr_vic[1],
                                          pos_dr_vic[2])
                    cf.commander.send_position_setpoint(setpoint[0],
                                                        setpoint[1],
                                                        setpoint[2],
                                                        setpoint[3])
                    time.sleep(0.5)

            cf.commander.send_stop_setpoint()
            time.sleep(0.5)
