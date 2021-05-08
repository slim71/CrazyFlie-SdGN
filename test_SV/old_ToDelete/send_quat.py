import time
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
import script_variables as sc_v
import script_setup as sc_s
import logging
import numpy as np
from vicon_dssdk import ViconDataStream
from own_module import crazyfun as crazy
import sequences as seq

with SyncCrazyflie(sc_v.uri, sc_s.cf) as scf:  # automatic connection
    scf.cf.param.set_value('stabilizer.estimator', 2)  # set KF as estimator

    log = crazy.datalog(scf)
    log.start()

    with MotionCommander(scf, sc_v.DEFAULT_HEIGHT) as mc:
        logging.info('===============Take-Off!================')

        crazy.matlab_print("% SQUARE")
        crazy.matlab_print("% x y z isPositionBlocked? "
                           "qx qy qz qw isOrientationBlocked?"
                           "setx_v sety_v setz_v"
                           "setx_cf sety_cf setz_cf")

        for point in seq.square:
            for i in range(10):
                try:  # make it try more than once?
                    logging.info("Getting a frame...")
                    sc_s.vicon.GetFrame()
                except ViconDataStream.DataStreamException as exc:
                    logging.error("Error while getting a frame in the core! "
                                  "--> %s", exc)

                sc_v.drone_pos = sc_s.vicon. \
                    GetSegmentGlobalTranslation(sc_v.drone, sc_v.drone)
                position = np.array(sc_v.drone_pos[0])
                logging.debug("Drone position: %s" + str(position))

                sc_v.drone_or = sc_s.vicon.\
                    GetSegmentGlobalRotationQuaternion(sc_v.drone, sc_v.drone)
                quaternion = np.array(sc_v.drone_or[0])
                # or we could use GetSegmentGlobalRotationEuler
                angles = np.rad2deg(crazy.euler_from_quaternion(
                    quaternion[0], quaternion[1],
                    quaternion[2], quaternion[3]))
                logging.debug("Quaternion for drone orientation: %s"
                              + str(quaternion))
                logging.debug("Converted in orientation: %s" + str(angles))

                scf.cf.extpos.send_extpose(position[0],
                                           position[1],
                                           position[2],
                                           quaternion[0],
                                           quaternion[1],
                                           quaternion[2],
                                           quaternion[3])
                setpoint = crazy.rot_with_quat(point, quaternion)
                scf.cf.commander.send_position_setpoint(setpoint[0],
                                                        setpoint[1],
                                                        setpoint[2],
                                                        angles[3])
                time.sleep(0.1)

                crazy.matlab_print(position[0], position[1], position[2],
                                   sc_v.drone_pos[1],
                                   quaternion[0], quaternion[1],
                                   quaternion[2], quaternion[2],
                                   sc_v.drone_or[1],
                                   point[0], point[1], point[2],
                                   setpoint[0], setpoint[1], setpoint[2])

    log.stop()
