from __future__ import division
from threading import Timer
import time
import logging
from own_module import crazyfun as crazy
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
import script_variables as sc_v
import script_setup as sc_s
from vicon_dssdk import ViconDataStream
import numpy as np
import sequences as seq

point_index = 0
go_on = 1


def send_position(sinc_cf, setp, index):
    point = setp[index]
    sinc_cf.cf.commander.send_position_setpoint(point[0],
                                                point[1],
                                                point[2],
                                                0)


def check_threshold(first, second):
    threshold = 0.05
    return (first[0] - second[0] <= threshold) and \
           (first[1] - second[1] <= threshold) and \
           (first[2] - second[2] <= threshold)


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

    initPos_V = (float(initPos_V[0]/1000),
                 float(initPos_V[1]/1000),
                 float(initPos_V[2]/1000))

    logging.debug("initial position %s", str(initPos_V))
    logging.debug("initial orientation %s", str(initOr_V))

    with MotionCommander(scf, sc_v.DEFAULT_HEIGHT) as mc:
        logging.info("Take-off!")

        setpoint = seq.square[point_index]
        Timer(0.2, send_position, [scf, setpoint, point_index]).start()

        # crazy.matlab_print("% SQUARE")
        crazy.matlab_print("% x y z  "
                           "qx qy qz qw "
                           "setx_v sety_v setz_v"
                           "setx_cf sety_cf setz_cf"
                           "timestamp")

        while go_on:
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
                                       current_quat[0],
                                       current_quat[1],
                                       current_quat[2],
                                       current_quat[3])

            time.sleep(0.1)

            crazy.matlab_print(sc_v.drone_pos[0], sc_v.drone_pos[1],
                               sc_v.drone_pos[2],
                               sc_v.drone_or[0], sc_v.drone_or[1],
                               sc_v.drone_or[2], sc_v.drone_or[2])

            if check_threshold(current_pos, setpoint):
                if point_index < setpoint.__len__() - 1:
                    point_index += 1
                elif point_index == setpoint.__len__() - 1:
                    break

    datalog.stop()
