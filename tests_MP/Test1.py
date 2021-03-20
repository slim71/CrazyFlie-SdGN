# come tutorial ma con classe logging; MC sia dentro che fuori dal metodo

from own_module.log_estimation_manager import LogEstimationManager
import time
import logging
from datetime import datetime
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from own_module.vicon_manager import ViconManager

URI = 'radio://0/80/2M'
DEFAULT_HEIGHT = 0.5
DESIRED_X = 0.5

global position_estimate, attitude_estimate

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
if __name__ == '__main__':

    with LogEstimationManager() as log_est:
        with SyncCrazyflie(URI) as scf:
            cf = scf.cf
            log_est.simple_log_async(cf)
            log_est.reset_estimator(cf)
            vicon = ViconManager()
            with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
                time.sleep(1)
                mc.start_back()
                while 1:
                    pos_dr_vic = vicon.get_drone_pos()
                    cf.extpos.send_extpos(pos_dr_vic[0],
                                          pos_dr_vic[1],
                                          pos_dr_vic[2])
                    position_estimate, attitude_estimate = log_est.get_estimation()
                    if position_estimate[0] > DESIRED_X:
                        mc.start_back()
                    elif position_estimate[0] < -DESIRED_X:
                        mc.start_forward()
                    time.sleep(0.1)
