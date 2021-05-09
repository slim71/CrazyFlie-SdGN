import time
import threading
import logging
import math
import numpy as np
from vicon_dssdk import ViconDataStream
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.position_hl_commander import PositionHlCommander
from own_module import crazyfun as crazy, script_setup as sc_s, \
    script_variables as sc_v

safety_threshold = 10  # cm
tv_prec = []
th_prec = []

# Class used to start the synchronization with the drone
with SyncCrazyflie(sc_v.uri, sc_s.cf) as scf:  # automatic connection
    scf.cf.param.set_value('stabilizer.estimator', 2)  # set KF as estimator
    scf.cf.param.set_value('commander.enHighLevel', '1')

    crazy.reset_estimator(scf)
    crazy.wait_for_position_estimator(scf)

    datalog = crazy.datalog(scf)
    datalog.start()

    time.sleep(0.5)

    crazy.int_matlab.write("% x y z qx qy qz qw")
    crazy.set_matlab.write("% set_x set_y set_z")

    est_thread = threading.Thread(target=crazy.repeat_fun,
                                  args=(crazy.vicon2drone_period,
                                        crazy.pose_sending, scf))
    with PositionHlCommander(
            scf,
            x=0.0, y=0.0, z=0.0,
            default_velocity=0.3,
            default_height=0.5,
            controller=PositionHlCommander.CONTROLLER_PID) as pc:

        logging.info('===============Take-Off!================')

        while 1:
            logging.info("Getting object setpoint...")
            obj_pos = sc_s.vicon. \
                GetSegmentGlobalTranslation('Obstacle', 'OneMarker')[0]

            dist_array = np.array(sc_v.drone_pos - obj_pos)

            theta_ver = math.atan2(dist_array[2], dist_array[0])
            theta_hor = math.atan2(dist_array[1], dist_array[0])

            # if it's not the first time the object has been registered
            if len(tv_prec) and len(th_prec):
                ver_warning = (0 < theta_ver < tv_prec[-1]) or\
                              (0 > theta_ver > tv_prec[-1])
                hor_warning = (0 < theta_hor < th_prec[-1]) or\
                              (0 > theta_hor > th_prec[-1])

                if ver_warning and hor_warning and\
                        (np.linalg.norm(dist_array) <= safety_threshold):
                    crazy.avoid(pc, dist_array)

            tv_prec.append(theta_ver)
            th_prec.append(theta_hor)
