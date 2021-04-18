import logging
import time
from cflib.positioning.position_hl_commander import PositionHlCommander
from own_module import crazyfun as crazy
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
import script_variables as sc_v
import script_setup as sc_s
import sequences as seq
from vicon_dssdk import ViconDataStream
import numpy as np

with SyncCrazyflie(sc_v.uri, sc_s.cf) as scf:
    scf.cf.param.set_value('stabilizer.estimator', 2)  # set KF as estimator
    crazy.reset_estimator(scf)
    # crazy.wait_for_position_estimator(scf)

    datalog = crazy.datalog(scf)
    datalog.start()

    time.sleep(0.5)

    # Get initial drone frame, both position and orientation, wrt Vicon frame
    initPos_V = sc_s.vicon.GetSegmentGlobalTranslation(sc_v.drone,
                                                       sc_v.drone)[0]
    initOr_V = sc_s.vicon.GetSegmentGlobalRotationQuaternion(sc_v.drone,
                                                             sc_v.drone)[0]

    initPos_V = (float(initPos_V[0] / 1000),
                 float(initPos_V[1] / 1000),
                 float(initPos_V[2] / 1000))

    logging.debug("initial position %s", str(initPos_V))
    logging.debug("initial orientation %s", str(initOr_V))

    with PositionHlCommander(
            scf,
            x=0.0, y=0.0, z=0.0,
            default_velocity=0.3,
            default_height=0.5,
            controller=PositionHlCommander.CONTROLLER_PID) as pc:

        logging.info("Take-off!")

        # crazy.matlab_print("% SQUARE")
        crazy.matlab_print("% x y z  "
                           "qx qy qz qw "
                           "setx_v sety_v setz_v"
                           "setx_cf sety_cf setz_cf")

        for point in seq.square:
            # transform the setpoint into drone initial frame
            setpoint_G, totalQuat = crazy. \
                to_drone_global(point, (0, 0, 0, 0),
                                initPos_V, initOr_V)

            finalSet_G = np.array((setpoint_G[0],
                                   setpoint_G[1],
                                   setpoint_G[2],
                                   0))

            logging.debug("original setpoint: %s", str(point))
            logging.debug("setpoint position in drone initial frame: %s",
                          str(setpoint_G[:3]))
            logging.debug("total orientation quaternion: %s",
                          str(totalQuat))
            logging.debug("setpoint to send: %s",
                          str(finalSet_G))

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

            sc_v.drone_pos = (float(sc_v.drone_pos[0] / 1000),
                              float(sc_v.drone_pos[1] / 1000),
                              float(sc_v.drone_pos[2] / 1000))

            # transform current pose in Drone initial frame
            current_pos, current_quat = crazy. \
                to_drone_global(sc_v.drone_pos, sc_v.drone_or,
                                initPos_V, initOr_V)

            est_pose = np.array((current_pos[0],
                                 current_pos[1],
                                 current_pos[2],
                                 crazy.quat2yaw(current_quat)))

            logging.debug("current position in drone initial frame: %s",
                          str(current_pos))
            logging.debug("current orientation quaternion: %s \n",
                          str(current_quat))

            # send current pose and setpoint, both in Drone initial frame
            # scf.cf.extpos.send_extpos(current_pos[0],
            #                           current_pos[1],
            #                           current_pos[2])
            scf.cf.extpos.send_extpose(current_pos[0],
                                       current_pos[1],
                                       current_pos[2],
                                       current_quat[0],
                                       current_quat[1],
                                       current_quat[2],
                                       current_quat[3])
            pc.got_to(finalSet_G[0],
                      finalSet_G[1],
                      finalSet_G[2])

            crazy.matlab_print(sc_v.drone_pos[0], sc_v.drone_pos[1],
                               sc_v.drone_pos[2],
                               sc_v.drone_or[0], sc_v.drone_or[1],
                               sc_v.drone_or[2], sc_v.drone_or[2],
                               point[0], point[1], point[2],
                               finalSet_G[0], finalSet_G[1], finalSet_G[2])

    datalog.stop()
