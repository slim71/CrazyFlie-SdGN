import time
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
import script_variables as sc_v
import script_setup as sc_s
import logging
import numpy as np
from vicon_dssdk import ViconDataStream

sequence = [
    (0.5, 0, sc_v.DEFAULT_HEIGHT),
    (0, 0, sc_v.DEFAULT_HEIGHT),
    (-0.5, 0, sc_v.DEFAULT_HEIGHT)
]


with SyncCrazyflie(sc_v.uri, sc_s.cf) as scf:  # automatic connection
    scf.cf.param.set_value('stabilizer.estimator', 2)  # set KF as estimator

    with MotionCommander(scf, sc_v.DEFAULT_HEIGHT) as mc:
        logging.info('===============Take-Off!================')

        try:
            sc_s.vicon.GetFrame()
        except ViconDataStream.DataStreamException as exc:
            logging.error("Error while getting a frame in the core! "
                          "--> %s", exc)

        sc_v.drone_pos = sc_s.vicon.\
            GetSegmentGlobalTranslation(sc_v.drone, sc_v.drone)[0]
        position = np.array(sc_v.drone_pos, dnmin=2)
        logging.debug("Drone position: %s" + str(position))

        orientation = sc_s.vicon.\
            GetSegmentGlobalRotationEulerXYZ(sc_v.drone, sc_v.drone)[0]
        logging.debug("Euler angles: %s" + str(orientation))

        sc_v.drone_or = sc_s.vicon.\
            GetSegmentGlobalRotationMatrix(sc_v.drone, sc_v.drone)[0]
        rot_matrix = np.mat(sc_v.drone_or)
        logging.debug("Rotation matrix: %s" + str(rot_matrix))

        rototrasl = np.concatenate(
            (np.concatenate((rot_matrix, position.T), axis=1),
             np.array([[0, 0, 0, 1]])), axis=0)
        logging.debug("Homogeneous matrix: %s" + str(rototrasl))

        for point in sequence:
            for i in range(10):
                try:
                    logging.info("Getting a frame...")
                    sc_s.vicon.GetFrame()
                except ViconDataStream.DataStreamException as exc:
                    logging.error("Error while getting a frame in the core! "
                                  "--> %s", exc)

                sc_v.drone_pos = sc_s.vicon. \
                    GetSegmentGlobalTranslation(sc_v.drone, sc_v.drone)[0]
                position = np.array(sc_v.drone_pos, dnmin=2)
                logging.debug("Drone position: %s" + str(position))

                reference = np.concatenate(
                    (np.array(point, ndmin=2).T, np.array([1], ndmin=2)),
                    axis=0)

                setpoint = np.matmul(rototrasl, reference)
                logging.debug("Setpoint after rotation: %s" + str(setpoint))

                scf.cf.extpos.send_extpos(position[0],
                                          position[1],
                                          position[2])
                scf.cf.commander.send_position_setpoint(setpoint[0, 3],
                                                        setpoint[1, 3],
                                                        setpoint[2, 3],
                                                        -orientation[2])
                time.sleep(0.1)
