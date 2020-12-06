# Module containing all the function used in the Crazyflie project

# TODO: complete functions help

# -----------------------------------------------------------------------------
# ----------------------------------IMPORT-------------------------------------
# -----------------------------------------------------------------------------
from __future__ import print_function
# TODO: LogConfig not public, so not imported?
from cflib.crazyflie.log import LogConfig
import numpy as np
import math
import time

# Max number of losses during acquisition of the position of the Wand or Drone
MAX_LOSS = 10


# TODO: module/global variables?


# -----------------------------------------------------------------------------
# ----------------------------------FUNCTIONS----------------------------------
# -----------------------------------------------------------------------------

# This function resets the Kalman Filter.
def reset_estimator(crazyflie):
    """
    Resets the estimator.

        :param crazyflie: object linked to the Crazyflie implementation
        :return: None

    """

    # The reset command is active on rising edge, so setting from 0 to 1 resets
    # the KF. We set it back to 0 so that next time the script is run it is
    # already reset.
    # https://forum.bitcraze.io/viewtopic.php?t=2746
    crazyflie.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    crazyflie.param.set_value('kalman.resetEstimation', '0')


def log_stab_callback(timestamp, data, log_conf):
    """
    Prints out data gathered from the LogTable.

        :param timestamp:
        :param data:
        :param log_conf:
        :return:

    """

    # TODO: global variables not declared at module level
    print('[%d][%s]: %s' % (timestamp, log_conf.name, data))
    global log_pos_x
    global log_pos_y
    global log_pos_z
    global log_roll
    global log_pitch
    global log_yaw

    # TODO: Setting (estimated?) state variables
    log_pos_x = data['stateEstimate.x']
    log_pos_y = data['stateEstimate.y']
    log_pos_z = data['stateEstimate.z']
    log_roll = data['stabilizer.roll']
    log_pitch = data['stabilizer.pitch']
    log_yaw = data['stabilizer.yaw']


def simple_log_async(sync_crazyflie, log_conf):
    """
    Adds a callback function to the LogTable of the Drone.

    :param sync_crazyflie: synchronization wrapper of the Crazyflie object
    :param log_conf:
    :return: None
    """

    cf = sync_crazyflie.cf
    cf.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(log_stab_callback)


def config_logging(sync_crazyflie):
    """
    Prepares the call to "simple_log_async" and achieves it.

    :param sync_crazyflie: synchronization wrapper of the Crazyflie object
    :return:
    """

    lg_stab = LogConfig(name='Stabilizer', period_in_ms=10)
    lg_stab.add_variable('stateEstimate.x', 'float')
    lg_stab.add_variable('stateEstimate.y', 'float')
    lg_stab.add_variable('stateEstimate.z', 'float')
    lg_stab.add_variable('stabilizer.roll', 'float')
    lg_stab.add_variable('stabilizer.pitch', 'float')
    lg_stab.add_variable('stabilizer.yaw', 'float')

    simple_log_async(sync_crazyflie, lg_stab)

    # TODO: returned parameter needed?
    return lg_stab


def getFirstPosition(vicon_client, drone_obj):
    """
    Gathers Drone position information from the Vicon client.

    :param vicon_client: Vicon client to connect to
    :param drone_obj: Drone considered
    :return:
    """

    drone_trans_m = np.array([0, 0, 0])
    iterations = 0

    # It can happen that the position value [0,0,0] is received
    # more than once. This means that something wrong is
    # happening with the Vicon Tracking.
    # So, in order to avoid errors, we count how many times we receive [0,0,0]
    # and if it exceeds MAX_LOSS we stop the experiment.
    while (drone_trans_m[0] == 0) and (drone_trans_m[1] == 0) and (
            drone_trans_m[2] == 0):
        # This call is necessary before each call to some
        # "GetSegment...()"-like function
        a = vicon_client.GetFrame()
        b = vicon_client.GetFrameNumber()

        drone_tuple = vicon_client.GetSegmentGlobalTranslation(drone_obj,
                                                               drone_obj)

        # We are interested only in the first part of this structure
        drone_trans_mm = drone_tuple[0]

        # Absolute position of the Drone converted from [mm] to [m]
        drone_trans_m = np.array([float(drone_trans_mm[0]) / 1000,
                                  float(drone_trans_mm[1]) / 1000,
                                  float(drone_trans_mm[2]) / 1000]
                                 )

        if iterations > MAX_LOSS:
            print(
                "WARNING: initial position of the Drone is [0,0,0] for ",
                MAX_LOSS,
                "consecutive times. Please try with another initial position "
                "and restart the experiment."
            )
            exit()
        else:
            iterations += 1

    return drone_trans_m


# This function creates the homogeneous matrix used for the
# conversion between the Navigation System and the Vicon System.
# The matrix is comprised of:
# - Rotation:    we assume always NULL values for the roll and pitch angles,
#                so the rotation is expressed with a 3x3 matrix on the Z-axis
#                with yaw angle "gamma"
# - Translation: it is the initial position of the Drone. It is rotated with
#                the previous matrix.
def createMatrixRotation(vicon_client, drone_obj, drone_trans_m, last_gamma,
                         KF_quat):
    """
    Creates the matrices used to map the transformation between
    Drone and Vicon reference systems.

    :param vicon_client: Vicon client
    :param drone_obj: Vicon name of the Drone
    :param drone_trans_m: translation of the Drone in the Vicon reference
            system
    :param last_gamma: last recorded value for the yaw angle
    :param KF_quat: flag to include quaternions into KF

    :return:
        HomMatrix: homogeneous transformation matrix to Drone body,
        RotMatrix: rotation matrix to Drone body,
        last_gamma: last valid yaw angle value TODO: in which system?
    """

    if KF_quat:
        # In this case we don't need to take in consideration the
        # orientation of the drone in the conversion between frames:f
        # this will be the Kalman filter's concern
        gamma = 0
        last_gamma = 0

    else:
        # Otherwise the K-F doesn't receive updates of the yaw angle from
        # us, so we take it in consideration in the Matrix
        # in order to correctly convert between frames.

        # We get the orientation of the Drone from which we want to get the
        # Yaw angle
        Euler = vicon_client. \
            GetSegmentGlobalRotationEulerXYZ(drone_obj, drone_obj)
        # We are interested only in the first part of the structure:
        Euler_radians = Euler[0]

        # In case we receive tha value [0,0,0], that means an error with the
        # Vicon Tracker, we continue using the previous value of the yaw angle
        if (Euler_radians[0] == 0
                and Euler_radians[1] == 0
                and Euler_radians[2] == 0):
            gamma = last_gamma
        else:
            gamma = Euler_radians[2]
            last_gamma = gamma

    # roll angle:
    alpha = 0
    # pitch angle:
    beta = 0

    # We build the homogeneous rotation matrix that will convert a
    # position expressed in the Vicon reference system
    # in position expressed in the Body frame of the Drone
    R_x = np.array([[1, 0, 0],
                    [0, math.cos(alpha), -math.sin(alpha)],
                    [0, math.sin(alpha), math.cos(alpha)]])
    R_y = np.array([[math.cos(beta), 0, math.sin(beta)],
                    [0, 1, 0],
                    [-math.sin(beta), 0, math.cos(beta)]])
    R_z = np.array([[math.cos(gamma), -math.sin(gamma), 0],
                    [math.sin(gamma), math.cos(gamma), 0],
                    [0, 0, 1]])
    Rot_xy = np.dot(R_x, R_y)  # R_x * R_y
    Rot_xyz = np.dot(Rot_xy, R_z)  # R_xy * R_z
    RotMatrix = np.transpose(Rot_xyz)  # R_xyz'
    HomMatrix = np.array([[RotMatrix[0][0],
                           RotMatrix[0][1],
                           RotMatrix[0][2],
                           -np.dot(RotMatrix,
                                   np.transpose(drone_trans_m))[0]
                           ],
                          [RotMatrix[1][0],
                           RotMatrix[1][1],
                           RotMatrix[1][2], -
                           np.dot(RotMatrix,
                                  np.transpose(drone_trans_m))[1]
                           ],
                          [RotMatrix[2][0],
                           RotMatrix[2][1],
                           RotMatrix[2][2], -
                           np.dot(RotMatrix,
                                  np.transpose(drone_trans_m))[2]
                           ],
                          [0, 0, 0, 1]])

    # In case we update the last value of the yaw angle (not null value) we
    # update returning it together with the matrices
    return HomMatrix, RotMatrix, last_gamma


def landing(last_pos, sub_height, de_height, crazyflie, lg_stab,
            fdesc, log_flag):
    """
    This function implements the landing phase.

    :param last_pos:
    :param sub_height: total meters subtracted to the drone height
    :param de_height: amount to subtract each time from the drone height
    :param crazyflie: Crazyflie object linked to the drone
    :param lg_stab:
    :param fdesc:
    :param log_flag: flag specifying if log information should be produced
    :return: None

    """

    # We gradually reduce the height setpoint
    while last_pos[2] - sub_height > 0:
        crazyflie.commander.send_position_setpoint(last_pos[0],
                                                   last_pos[1],
                                                   last_pos[2] - sub_height,
                                                   0)
        sub_height = sub_height + de_height

    if log_flag:
        # close the connection with the log table:
        lg_stab.stop()
        fdesc.close()

    exit()

    return
