# Module containing all the function used in the Crazyflie project

# TODO: complete functions help

# -----------------------------------------------------------------------------
# ----------------------------------IMPORT-------------------------------------
# -----------------------------------------------------------------------------
from __future__ import print_function
import logging
import os
from datetime import datetime
from pathlib import Path
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.crazyflie.log import LogConfig
import numpy as np
import math
import time

# Max number of consecutive loss allowed during acquisition of the setpoint
# of the Wand or drone
MAX_LOSS = 10

log_pos_x = 0
log_pos_y = 0
log_pos_z = 0
log_roll = 0
log_pitch = 0
log_yaw = 0

# Security offset
SEC_OFFSET = 0.3  # [m]

# The height the drone has to reach at the end of the take-off. This can't
# be higher than the "MC_HEIGHT" used in the class of the
# Motion Commander. We suggest to set it at least at 90% of its value.
DEFAULT_HEIGHT = 0.5  # [m]

# This is the default height used by the Motion Commander during take-off.
# It has to be higher than DEFAULT_HEIGHT because we consider the
# take-off phase concluded once the drone reaches DEFAULT_HEIGHT, but this
# can't be always true because the Vicon might observe another value due to
# noise
MC_HEIGHT = 0.8  # [m]

# Number of frames for the Vicon buffer
frame_num = 1000

# -----------------------------------------------------------------------------
# ----------------------------------FUNCTIONS----------------------------------
# -----------------------------------------------------------------------------


# This function resets the Kalman Filter.
# TODO: already in the Crazyflie object!
def reset_estimator(crazyflie):
    """
    Resets the estimator.

        :param crazyflie: Object linked to the Crazyflie
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

        :param timestamp: Timestamp for the log file
        :param data: Data to be logged
        :param log_conf: Representation of a log configuration
        :return:None

    """

    print('[%d][%s]: %s' % (timestamp, log_conf.name, data))
    global log_pos_x
    global log_pos_y
    global log_pos_z
    global log_roll
    global log_pitch
    global log_yaw

    # Setting estimated state variables
    log_pos_x = data['stateEstimate.x']
    log_pos_y = data['stateEstimate.y']
    log_pos_z = data['stateEstimate.z']
    log_roll = data['stabilizer.roll']
    log_pitch = data['stabilizer.pitch']
    log_yaw = data['stabilizer.yaw']


def simple_log_async(sync_crazyflie, log_conf):
    """
    Adds a callback function to the LogTable of the drone.

    :param sync_crazyflie: Synchronization wrapper of the Crazyflie object
    :param log_conf: Representation of a log configuration
    :return: None
    """

    crazyflie = sync_crazyflie.cf
    crazyflie.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(log_stab_callback)


def config_logging(sync_crazyflie):
    """
    Prepares the call to "simple_log_async" and achieves it.
    Adds a log configuration to the Crazyflie with desired variables
    logged every 10ms.

    :param sync_crazyflie: Synchronization wrapper of the Crazyflie object
    :return: log_stab: Log stabilizer
    """

    log_stab = LogConfig(name='Stabilizer', period_in_ms=10)
    log_stab.add_variable('stateEstimate.x', 'float')
    log_stab.add_variable('stateEstimate.y', 'float')
    log_stab.add_variable('stateEstimate.z', 'float')
    log_stab.add_variable('stabilizer.roll', 'float')
    log_stab.add_variable('stabilizer.pitch', 'float')
    log_stab.add_variable('stabilizer.yaw', 'float')

    simple_log_async(sync_crazyflie, log_stab)

    return log_stab


def print_callback(timestamp, data, log_conf):
    """
    Prints out gathered data.

        :param timestamp: Timestamp for the log file
        :param data: Data to be logged
        :param log_conf: Representation of a log configuration
        :return:None

    """

    data_file = "../test_SV/data_logs/" + datetime.now().strftime("__%Y%m%d_%H%M")
    data_file = data_file + ".txt"

    ff = os.path.normpath(os.path.join(Path(__file__).parent.absolute(),
                                       data_file))
    with open(ff, 'a') as descriptor:
        # Setting estimated state variables
        pos_x = data['stateEstimate.x']
        pos_y = data['stateEstimate.y']
        pos_z = data['stateEstimate.z']
        roll = data['stabilizer.roll']
        pitch = data['stabilizer.pitch']
        yaw = data['stabilizer.yaw']

        print(pos_x, pos_y, pos_z, roll, pitch, yaw, file=descriptor)
        print(pos_x, pos_y, pos_z, roll, pitch, yaw)


def data_log_async(sync_crazyflie, log_conf):
    """
    Adds a callback function to the LogTable of the drone.

    :param sync_crazyflie: Synchronization wrapper of the Crazyflie object
    :param log_conf: Representation of a log configuration
    :return: None
    """

    crazyflie = sync_crazyflie.cf
    crazyflie.log.add_config(log_conf)
    log_conf.data_received_cb.add_callback(print_callback)


def datalog(sync_crazyflie):
    """
    Prepares the call to "simple_log_async" and achieves it.
    Adds a log configuration to the Crazyflie with desired variables
    logged every 10ms.

    :param sync_crazyflie: Synchronization wrapper of the Crazyflie object
    :return: log_stab: Log stabilizer
    """

    measure_log = LogConfig(name='TotalEstimate', period_in_ms=10)
    measure_log.add_variable('stateEstimate.x', 'float')
    measure_log.add_variable('stateEstimate.y', 'float')
    measure_log.add_variable('stateEstimate.z', 'float')
    measure_log.add_variable('stabilizer.roll', 'float')
    measure_log.add_variable('stabilizer.pitch', 'float')
    measure_log.add_variable('stabilizer.yaw', 'float')

    data_log_async(sync_crazyflie, measure_log)

    return measure_log


def getFirstPosition(vicon_client, drone_obj):
    """
    Gathers drone setpoint information from the Vicon client.

    :param vicon_client: Vicon client to connect to
    :param drone_obj: Drone considered
    :return drone_pos_m: Drone setpoint in the Vicon system, in meters
    """

    drone_pos_m = np.array([0.0, 0.0, 0.0])
    iterations = 0

    # It can happen that the setpoint value [0,0,0] is received
    # more than once. This means that something wrong is
    # happening with the Vicon Tracking.
    # So, in order to avoid errors, we count how many times we receive [0,0,0]
    # and if it exceeds MAX_LOSS we stop the experiment.
    while (drone_pos_m[0] == 0) \
            and (drone_pos_m[1] == 0) \
            and (drone_pos_m[2] == 0):
        # This call is necessary before each call to some
        # "GetSegment...()"-like function
        a = vicon_client.GetFrame()
        b = vicon_client.GetFrameNumber()

        drone_tuple = vicon_client. \
            GetSegmentGlobalTranslation(drone_obj, drone_obj)

        # We are interested only in the first part of this structure
        drone_pos_mm = drone_tuple[0]

        # Absolute setpoint of the drone converted from [mm] to [m]
        drone_pos_m = np.array([float(drone_pos_mm[0]) / 1000,
                                float(drone_pos_mm[1]) / 1000,
                                float(drone_pos_mm[2]) / 1000]
                               )

        if iterations > MAX_LOSS:
            print("WARNING: initial setpoint of the drone reported ",
                  "[0,0,0] for ",
                  MAX_LOSS,
                  "consecutive times. Please try with another initial "
                  "setpoint and restart the experiment."
                  )
            exit()
        else:
            iterations += 1

    return drone_pos_m


def createMatrixRotation(vicon_client, drone_obj, drone_trans_m, last_gamma,
                         kf_quat):
    """
    Creates the homogeneous matrix used for the conversion to the
    Navigation system (corresponding to the drone initial setpoint before
    take-off) from the Global system (Vicon reference system).
    The matrix is comprised of:
    - Rotation:   we assume always NULL values for the roll and pitch angles,
                  so the rotation is on the Z-axis with yaw angle "gamma",
                  expressed with a 3x3 matrix
    - Translation: it's the initial setpoint of the drone. It is rotated with
                   the previous matrix.

    :param vicon_client: Vicon client
    :param drone_obj: Vicon name of the drone
    :param drone_trans_m: Translation of the drone in the Vicon reference
            system
    :param last_gamma: Last recorded value for the yaw angle
    :param kf_quat: Flag to include quaternions into KF

    :return:
        HomMatrix: Homogeneous transformation matrix to drone body,
        last_gamma: Last valid yaw angle value,
        RotMatrix: Rotation matrix to drone body
    """

    if kf_quat:
        # In this case we don't need to take in consideration the
        # orientation of the drone in the conversion between frames:f
        # this will be the Kalman filter's concern
        gamma = 0
        last_gamma = 0

    else:
        # Otherwise the K-F doesn't receive updates of the yaw angle from
        # us, so we take it in consideration in the Matrix
        # in order to correctly convert between frames.

        # We get the orientation of the drone from which we want to get the
        # Yaw angle
        euler = vicon_client. \
            GetSegmentGlobalRotationEulerXYZ(drone_obj, drone_obj)
        # We are interested only in the first part of the structure:
        euler_radians = euler[0]
        logging.debug("Euler angles: %s", str(euler_radians))

        # In case we receive tha value [0,0,0], that means an error with the
        # Vicon Tracker, we continue using the previous value of the yaw angle
        if (euler_radians[0] == 0
                and euler_radians[1] == 0
                and euler_radians[2] == 0):
            gamma = last_gamma
        else:
            gamma = euler_radians[2]
            last_gamma = gamma

    # Roll angle:
    alpha = 0
    # Pitch angle:
    beta = 0

    # Build the homogeneous rotation matrix that will convert a
    # setpoint expressed in the Vicon reference system
    # in setpoint expressed in the Body frame of the drone
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
    HomMatrix = np.array([[RotMatrix[0, 0],
                           RotMatrix[0, 1],
                           RotMatrix[0, 2],
                           -np.dot(RotMatrix,
                                   np.transpose(drone_trans_m))[0]
                           ],
                          [RotMatrix[1, 0],
                           RotMatrix[1, 1],
                           RotMatrix[1, 2], -
                           np.dot(RotMatrix,
                                  np.transpose(drone_trans_m))[1]
                           ],
                          [RotMatrix[2, 0],
                           RotMatrix[2, 1],
                           RotMatrix[2, 2], -
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
    :param sub_height: Total meters subtracted to the drone height
    :param de_height: Amount to subtract each time from the drone height
    :param crazyflie: Crazyflie object linked to the drone
    :param lg_stab: Log stabilizer
    :param fdesc: Descriptor of the log file
    :param log_flag: Flag specifying if log info should be produced
    :return: None

    """

    # Gradually reduce the height setpoint
    while last_pos[2] - sub_height > 0:
        crazyflie.commander.send_position_setpoint(last_pos[0],
                                                   last_pos[1],
                                                   last_pos[2] - sub_height,
                                                   0)
        sub_height = sub_height + de_height

    if log_flag:
        # Close the connection with the log table
        lg_stab.stop()
        fdesc.close()

    exit()

    return


def wait_for_position_estimator(scf):
    logging.info('Waiting for estimator to find setpoint...')

    log_config = LogConfig(name='Kalman Variance', period_in_ms=500)
    log_config.add_variable('kalman.varPX', 'float')
    log_config.add_variable('kalman.varPY', 'float')
    log_config.add_variable('kalman.varPZ', 'float')

    var_x_history = [1000] * 10
    var_y_history = [1000] * 10
    var_z_history = [1000] * 10

    threshold = 0.001

    with SyncLogger(scf, log_config) as logger:
        for log_entry in logger:
            data = log_entry[1]

            var_x_history.append(data['kalman.varPX'])
            var_x_history.pop(0)
            var_y_history.append(data['kalman.varPY'])
            var_y_history.pop(0)
            var_z_history.append(data['kalman.varPZ'])
            var_z_history.pop(0)

            min_x = min(var_x_history)
            max_x = max(var_x_history)
            min_y = min(var_y_history)
            max_y = max(var_y_history)
            min_z = min(var_z_history)
            max_z = max(var_z_history)

            logging.debug("dx: %s dy: %s dz: %s",
                          max_x - min_x, max_y - min_y, max_z - min_z)

            if (max_x - min_x) < threshold and (
                    max_y - min_y) < threshold and (
                    max_z - min_z) < threshold:
                break


def sign(x):
    if x > 0:
        return +1
    elif x < 0:
        return -1
    elif x == 0:
        return 0
    else:
        raise Exception("The argument is not a number.")


def avoid(vehicle, dist):
    movement = np.array([0, 0, 0])

    direction = min(dist)
    ind = dist.index(direction)

    movement[ind] = -1*sign(direction)*0.5

    # might have to get rotation matrix
    # might be usefule to get the Euler angles
    vehicle.commander.send_position_setpoint(movement[0],
                                             movement[1],
                                             movement[2],
                                             0)
    time.sleep(0.1)


def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z  # in radians
