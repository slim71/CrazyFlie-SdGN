import logging
import time
from own_module import crazyfun as crazy
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
import script_variables as sc_v
import script_setup as sc_s
import sequences as seq
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
    initOr_V = sc_s.vicon.GetSegmentGlobalRotationQuaternion(sc_v.drone,
                                                             sc_v.drone)[0]

    logging.debug("initial position %s, initial orientation %s",
                  str(initPos_V), str(initOr_V))

    with MotionCommander(scf, sc_v.DEFAULT_HEIGHT) as mc:
        logging.info("Take-off!")

        # Select setpoint sequence to send
        to_fly = seq.square

        # for each setpoint in sequence
        for point in to_fly:

            # quat_VtoG = np.array((-initOr_V[0], -initOr_V[1],
            #                       -initOr_V[2], initOr_V[3]))
            # quat_VtoG_conj = np.array(initOr_V)
            #
            # pointPos_V = np.array((point[0], point[1], point[2], 0))
            # Gpos_V = np.array((initPos_V[0], initPos_V[1], initPos_V[2], 0))
            #
            # pointPos_G = crazy.\
            #     quat_product(quat_VtoG,
            #                  crazy.quat_product(pointPos_V - Gpos_V,
            #                                     quat_VtoG_conj))
            #
            # setpoint_G = np.array((float(pointPos_G[0]/1000),
            #                        float(pointPos_G[1]/1000),
            #                        float(pointPos_G[2]/1000)))
            # totalQuat = crazy.quat_product(quat_VtoG, initOr_V)

            # transform the setpoint into drone initial frame
            setpoint_G, totalQuat = crazy.\
                to_drone_global(point, crazy.yaw2quat(point[3]),
                                initPos_V, initOr_V)

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

            for i in range(20):
                try:
                    sc_s.vicon.GetFrame()
                except ViconDataStream.DataStreamException as exc:
                    logging.error("Error while getting a frame in the core! "
                                  "--> %s", str(exc))

                sc_v.drone_pos = sc_s.vicon. \
                    GetSegmentGlobalTranslation(sc_v.drone, sc_v.drone)[0]
                sc_v.drone_or = sc_s.vicon. \
                    GetSegmentGlobalRotationQuaternion(sc_v.drone,
                                                       sc_v.drone)[0]

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
                scf.cf.commander.send_position_setpoint(finalSet_G[0],
                                                        finalSet_G[1],
                                                        finalSet_G[2],
                                                        finalSet_G[3])
                time.sleep(0.5)

    datalog.stop()
