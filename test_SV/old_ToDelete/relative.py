import logging
import time
from own_module import crazyfun as crazy
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
import script_variables as sc_v
import script_setup as sc_s
from vicon_dssdk import ViconDataStream
import numpy as np

with SyncCrazyflie(sc_v.uri, sc_s.cf) as scf:
    scf.cf.param.set_value('stabilizer.estimator', 2)  # set KF as estimator
    crazy.wait_for_position_estimator(scf)

    datalog = crazy.datalog(scf)
    datalog.start()

    time.sleep(0.5)

    # Get initial drone frame, both position and orientation, wrt Vicon frame
    initPos_V = sc_s.vicon.GetSegmentGlobalTranslation(sc_v.drone,
                                                       sc_v.drone)[0]
    initOr_V = sc_s.vicon.GetSegmentaGlobalRotationQuaternion(sc_v.drone,
                                                              sc_v.drone)[0]

    logging.debug("initial position %s, initial orientation %s",
                  str(initPos_V), str(initOr_V))

    with MotionCommander(scf, sc_v.DEFAULT_HEIGHT) as mc:
        logging.info("Take-off!")

        # continuosly
        while 1:

            # get Wand position: it'll be the setpoint
            WandPos_V = sc_s.vicon.GetSegmentGlobalTranslation(sc_v.Wand,
                                                               sc_v.Wand)[0]
            WandQuat = sc_s.vicon.\
                GetSegmentGlobalRotationQuaternion(sc_v.Wand, sc_v.Wand)[0]

            # transform the setpoint into drone initial frame
            setpoint_G, totalQuat = crazy.to_drone_global(WandPos_V, WandQuat,
                                                          initPos_V, initOr_V)
            # transform the setpoint relatively to the Vicon origin
            # setpoint_G, totalQuat = crazy.to_drone_global(WandPos_V,
            #                                               WandQuat,
            #                                               initPos_V,
            #                                               initOr_V)

            finalSet_G = np.array((setpoint_G[0],
                                   setpoint_G[1],
                                   setpoint_G[2],
                                   crazy.quat2yaw(totalQuat)))

            logging.debug("setpoint position in drone initial frame: %s",
                          str(setpoint_G[:2]))
            logging.debug("total orientation quaternion: %s",
                          str(totalQuat))
            logging.debug("setpoint to send: %s2",
                          str(finalSet_G))

            # send setpoint more than once to keep the drone flying
            for i in range(20):
                # get a new frame
                try:
                    sc_s.vicon.GetFrame()
                except ViconDataStream.DataStreamException as exc:
                    logging.error("Error while getting a frame in the core! "
                                  "--> %s", str(exc))

                # get current position and orientation in Vicon
                sc_v.drone_pos = sc_s.vicon. \
                    GetSegmentGlobalTranslation(sc_v.drone, sc_v.drone)[0]
                sc_v.drone_or = sc_s.vicon. \
                    GetSegmentGlobalRotationQuaternion(sc_v.drone,
                                                       sc_v.drone)[0]

                # transform current pose in Drone initial frame
                current_pos, current_quat = crazy.\
                    to_drone_global(sc_v.drone_pos, sc_v.drone_or,
                                    initPos_V, initOr_V)

                est_pose = np.array((current_pos[0],
                                     current_pos[1],
                                     current_pos[2],
                                     crazy.quat2yaw(current_quat)))

                logging.debug("current position in drone initial frame: %s",
                              str(current_pos))
                logging.debug("current orientation quaternion: %s",
                              str(current_quat))

                # send current pose and setpoint, both in Drone initial frame
                # scf.cf.extpos.send_extpos(current_pos[0],
                #                           current_pos[1],
                #                           current_pos[2])
                scf.cf.extpos.send_extpose(current_pos[0],
                                           current_pos[1],
                                           current_pos[2],
                                           current_quat.drone_or[0],
                                           current_quat.drone_or[1],
                                           current_quat.drone_or[2],
                                           current_quat.drone_or[3])
                scf.cf.commander.send_position_setpoint(-finalSet_G[0],
                                                        -finalSet_G[1],
                                                        finalSet_G[2],
                                                        0)
                time.sleep(0.5)

    datalog.stop()
