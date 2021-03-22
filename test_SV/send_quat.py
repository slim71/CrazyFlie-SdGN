import time
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
import script_variables as sc_v
import script_setup as sc_s
import logging
import numpy as np
from vicon_dssdk import ViconDataStream
from own_module import crazyfun as crazy

with SyncCrazyflie(sc_v.uri, sc_s.cf) as scf:  # automatic connection
    scf.cf.param.set_value('stabilizer.estimator', 2)  # set KF as estimator

    with MotionCommander(scf, sc_v.DEFAULT_HEIGHT) as mc:
        logging.info('===============Take-Off!================')

        for i in range(10):
            try:
                logging.info("Getting a frame...")
                sc_s.vicon.GetFrame()
            except ViconDataStream.DataStreamException as exc:
                logging.error("Error while getting a frame in the core! "
                              "--> %s", exc)

            sc_v.drone_pos = sc_s.vicon. \
                GetSegmentGlobalTranslation(sc_v.drone, sc_v.drone)[0]
            position = np.array(sc_v.drone_pos)
            logging.debug("Drone position: %s" + str(position))

            quaternion = sc_s.vicon.\
                GetSegmentGlobalRotationQuaternion(sc_v.drone, sc_v.drone)
            logging.debug("Quaternion for drone orientation: %s"
                          + str(quaternion))
            logging.debug("Converted in orientation: %s" + str(np.rad2deg(
                crazy.euler_from_quaternion(quaternion[0],
                                            quaternion[1],
                                            quaternion[2],
                                            quaternion[3]))))

            scf.cf.extpos.send_extpose(position[0],
                                       position[1],
                                       position[2],
                                       quaternion[0],
                                       quaternion[1],
                                       quaternion[2],
                                       quaternion[3])
            time.sleep(0.1)
