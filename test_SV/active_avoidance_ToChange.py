import time
import math
from vicon_dssdk import ViconDataStream
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
import numpy as np
import logging
from own_module import crazyfun as crazy, script_setup as sc_s, \
    script_variables as sc_v

safety_threshold = 10  # cm
tv_prec = []
th_prec = []

# Class used to start the synchronization with the drone
with SyncCrazyflie(sc_v.uri, sc_s.cf) as scf:  # automatic connection
    logging.info("Connected!")

    scf.cf.param.set_value('stabilizer.estimator', 2)  # set KF as estimator

    # AUTO TAKE-OFF to sc_v.DEFAULT_HEIGHT!
    # Class used for the setpoint control during the take-off phase:
    # take-off automatic when context created using "with"
    with MotionCommander(scf, sc_v.DEFAULT_HEIGHT) as mc:
        logging.info('===============Take-Off!================')

        while 1:
            logging.info("Getting a frame during flight...")
            try:
                sc_v.got_frame = 0
                while not sc_v.got_frame:
                    got_frame = sc_s.vicon.GetFrame()
                    n_frame = sc_s.vicon.GetFrameNumber()
                    logging.debug("Fetched! Pulled frame number: %d"
                                  if sc_v.got_frame
                                  else "Vicon is not streaming!", n_frame)
            except ViconDataStream.DataStreamException as exc:
                logging.error("Error! --> %s", exc)
                exit("There was an error...")

            logging.info("Getting drone setpoint...")
            try:
                sc_v.drone_pos = sc_s.vicon.\
                    GetSegmentGlobalTranslation(sc_v.drone, 'Crazyflie')
                logging.info("Done.")
            except ViconDataStream.DataStreamException as exc:
                logging.error("Error while getting Drone setpoint! --> %s",
                              exc)
                exit("Error while getting Drone setpoint: " + str(exc))
            sc_v.drone_pos = sc_v.drone_pos[0]
            # sc_v.drone_pos_m = np.array([float(sc_v.drone_pos_m[0]) / 1000,
            #                              float(sc_v.drone_pos_m[1]) / 1000,
            #                              float(sc_v.drone_pos_m[2]) / 1000])

            sc_s.cf.extpos.send_extpos(sc_v.drone_pos[0],
                                       sc_v.drone_pos[1],
                                       sc_v.drone_pos[2])
            time.sleep(0.1)

            logging.info("Getting object setpoint...")
            try:
                obj_pos = sc_s.vicon. \
                    GetSegmentGlobalTranslation('Obstacle', 'OneMarker')
                logging.info("Done.")
            except ViconDataStream.DataStreamException as exc:
                logging.error("Error while getting Object setpoint! --> %s",
                              exc)
                exit("Error while getting Object setpoint: " + str(exc))
            obj_pos = obj_pos[0]

            dist_array = np.array(sc_v.drone_pos - obj_pos)

            theta_ver = math.atan2(dist_array[2], dist_array[0])
            theta_hor = math.atan2(dist_array[1], dist_array[0])

            # if it's not the first time the ojbect has been registered
            if len(tv_prec) and len(th_prec):
                ver_warning = (0 < theta_ver < tv_prec[-1]) or\
                              (0 > theta_ver > tv_prec[-1])
                hor_warning = (0 < theta_hor < th_prec[-1]) or\
                              (0 > theta_hor > th_prec[-1])

                if ver_warning and hor_warning and\
                        (np.linalg.norm(dist_array) <= safety_threshold):
                    crazy.avoid(sc_v.drone, dist_array)

            tv_prec.append(theta_ver)
            th_prec.append(theta_hor)
