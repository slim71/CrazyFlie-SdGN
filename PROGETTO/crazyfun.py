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


# -----------------------------------------------------------------------------
# ----------------------------------FUNCTIONS----------------------------------
# -----------------------------------------------------------------------------


# This function resets the Kalman filter (before flying).
def reset_estimator(cf):
    """
    Resets the estimator.

        :param cf:
        :return:

    """

    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')
    # TODO: why double assignment?


# This is the function used for consulting the log table
# in the function "simple_log_async"
def log_stab_callback(timestamp, data, logconf):
    """
    Prints out data gathered from the specified

        :param timestamp:
        :param data:
        :param logconf:
        :return:

    """

    # TODO: global variables not declared at module level
    print('[%d][%s]: %s' % (timestamp, logconf.name, data))
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


# This function is used in "config_logging" and it configures the log table
# of the Drone adding the callback for the desired parameters.
def simple_log_async(scf, logconf):
    """
    Adds a callback function to the logtable of the Drone.

    :param scf:
    :param logconf:
    :return:
    """
    cf = scf.cf
    cf.log.add_config(logconf)
    logconf.data_received_cb.add_callback(log_stab_callback)


# This is used to prepare the input argument of the "simple_log_async" and to
# call it.
def config_logging(scf):
    """
    Prepares the function "simple_log_async" and calls it.

    :param scf:
    :return:
    """

    lg_stab = LogConfig(name='Stabilizer', period_in_ms=10)
    lg_stab.add_variable('stateEstimate.x', 'float')
    lg_stab.add_variable('stateEstimate.y', 'float')
    lg_stab.add_variable('stateEstimate.z', 'float')
    lg_stab.add_variable('stabilizer.roll', 'float')
    lg_stab.add_variable('stabilizer.pitch', 'float')
    lg_stab.add_variable('stabilizer.yaw', 'float')

    simple_log_async(scf, lg_stab)

    # TODO: returned parameter needed?
    return lg_stab


# This function gets the position of the Drone from the Vicon client (Tracker).
# It is used only for the first value of position, because it is the only one
# we need to define the translation between the Navigation System (defined
# when the Drone is turned on) and the Vicon System (fixed in the center of
# the flight room).
def get_First_Position(client, Drone):
    """
    Gathers Drone position information from the Vicon client.

    :param client: Vicon client to connect to
    :param Drone: Drone considered
    :return:
    """

    D_T_meters = np.array([0, 0, 0])
    iterations = 0

    # It can happen that the position value [0,0,0] is received
    # more than once. This means that something wrong is
    # happening with the Vicon Tracking.
    # So, in order to avoid errors, we count how many times we receive [0,0,0]
    # and if it exceeds MAX_LOSS we stop the experiment.
    while (D_T_meters[0] == 0) and (D_T_meters[1] == 0) and (
            D_T_meters[2] == 0):
        # This call is necessary before each call to some
        # "GetSegment...()"-like function
        a = client.GetFrame()
        b = client.GetFrameNumber()

        D_T_tuple = client.GetSegmentGlobalTranslation(Drone, Drone)

        # We are interested only in the first part of this structure
        D_T_millimeters = D_T_tuple[0]

        # Absolute position of the Drone converted from [mm] to [m]
        D_T_meters = np.array(
            [float(D_T_millimeters[0]) / 1000,
             float(D_T_millimeters[1]) / 1000,
             float(D_T_millimeters[2]) / 1000]
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

    return D_T_meters


# This function creates the homogeneous matrix used for the
# conversion between the Navigation System and the Vicon System.
# The matrix is comprised of:
# - Rotation:    we assume always NULL values for the roll and pitch angles,
#                so the rotation is expressed with a 3x3 matrix on the Z-axis
#                with yaw angle "gamma"
# - Translation: it is the initial position of the Drone. It is rotated with
#                the previous matrix.
def create_Matrix_Rotation(client, Drone, D_T_meters, last_gamma):
    """
    Creates the homogeneous matrix to convert between the Navigation
    reference
    system and the Vicon reference system.
    The included rotation is hypothesized with just a Yaw angle.
    TODO: add specifically from which one to which other.

    :param client: Vicon client to connect to
    :param Drone: Drone considered
    :param D_T_meters:
    :param last_gamma: last valid value received for the yaw angle
    :return:
    """

    # We get the orientation of the Drone, to derive the Yaw angle
    Euler_XYZ = client.GetSegmentGlobalRotationEulerXYZ(Drone, Drone)

    # We are interested only in the first part of the structure
    Euler_radians = Euler_XYZ[0]

    # In case we receive tha value [0,0,0] (that means an error with the
    # Vicon Tracker) we continue using the previous value of the yaw angle.
    if (Euler_radians[0] == 0) and (Euler_radians[1] == 0) \
            and (Euler_radians[2] == 0):

        gamma = last_gamma
    else:
        gamma = Euler_radians[2]
        last_gamma = gamma

    # Roll angle:
    alpha = 0
    # Pitch angle:
    beta = 0

    # Rotation matrices
    R_x = np.array([[1, 0, 0],
                    [0, math.cos(alpha), -math.sin(alpha)],
                    [0, math.sin(alpha), math.cos(alpha)]])
    R_y = np.array([[math.cos(beta), 0, math.sin(beta)],
                    [0, 1, 0],
                    [-math.sin(beta), 0, math.cos(beta)]])
    R_z = np.array([[math.cos(gamma), -math.sin(gamma), 0],
                    [math.sin(gamma), math.cos(gamma), 0],
                    [0, 0, 1]])

    # We build the homogeneous matrix that will convert the
    # positions expressed in the Vicon reference system
    # to the Body Frame of the Drone
    Matrix_Rotation_xy = np.dot(R_x, R_y)
    Matrix_Rotation_xyz = np.dot(Matrix_Rotation_xy, R_z)
    Matrix_Rotation = np.transpose(Matrix_Rotation_xyz)

    # TODO: maybe preemptively compute translational part and then only
    # select the desired elements here?
    Matrix_homogeneous = np.array(
        [[Matrix_Rotation[0][0], Matrix_Rotation[0][1], Matrix_Rotation[0][2],
          -np.dot(Matrix_Rotation, np.transpose(D_T_meters))[0]],
         [Matrix_Rotation[1][0], Matrix_Rotation[1][1], Matrix_Rotation[1][2],
          -np.dot(Matrix_Rotation, np.transpose(D_T_meters))[1]],
         [Matrix_Rotation[2][0], Matrix_Rotation[2][1], Matrix_Rotation[2][2],
          -np.dot(Matrix_Rotation, np.transpose(D_T_meters))[2]],
         [0, 0, 0, 1]])

    # In case we got a valid value of the yaw angle, we update it returning it
    # together with the matrix:
    return Matrix_homogeneous, last_gamma


def create_Matrix_Rotation(client, Drone, D_T_meters, last_gamma):
    #We get the start orientation of the Drone from which we want to get the RPY angles
    Euler_XYZ= client.GetSegmentGlobalRotationEulerXYZ(Drone, Drone)
    Euler_radians = Euler_XYZ[0]                #We are interested only in the first part of the tuple
    print("EULER Angles:", Euler_radians)
    if(Euler_radians[0] == 0 and Euler_radians[1] == 0 and Euler_radians[2] == 0 ):
        gamma = last_gamma
    else:
        gamma = Euler_radians[2]
        last_gamma = gamma

    alpha = 0                                  #We are interested only at the yaw angle
    beta = 0
    #From angles we build the homogeneous rotation matrix that will convert the positions expressed in the Vicon reference system in positions expressed in the body frame of the Drone
    R_x = np.array([[1 , 0, 0], [0, math.cos(alpha), -math.sin(alpha)], [0, math.sin(alpha), math.cos(alpha)]])
    R_y = np.array([[math.cos(beta), 0, math.sin(beta)], [0, 1, 0], [-math.sin(beta), 0, math.cos(beta)]])
    R_z = np.array([[math.cos(gamma), -math.sin(gamma), 0], [math.sin(gamma), math.cos(gamma), 0], [0, 0, 1]])
    Matrix_Rotation_xy = np.dot(R_x, R_y)
    Matrix_Rotation_xyz = np.dot(Matrix_Rotation_xy, R_z)
    Matrix_Rotation = np.transpose(Matrix_Rotation_xyz)
    Matrix_homogeneous= np.array([[Matrix_Rotation[0][0], Matrix_Rotation[0][1], Matrix_Rotation[0][2], -np.dot(Matrix_Rotation, np.transpose(D_T_meters))[0]], [Matrix_Rotation[1][0], Matrix_Rotation[1][1], Matrix_Rotation[1][2], -np.dot(Matrix_Rotation, np.transpose(D_T_meters))[1]], [Matrix_Rotation[2][0], Matrix_Rotation[2][1], Matrix_Rotation[2][2], -np.dot(Matrix_Rotation, np.transpose(D_T_meters))[2]], [0, 0, 0, 1]])
    #print("Matrix Homogenous:", Matrix_homogeneous)

    return Matrix_homogeneous, Matrix_Rotation, last_gamma


# This function creates the homogeneous matrix used for the
# conversion between the Navigation System and the Vicon System.
# The matrix is comprised of:
# - Rotation:    we assume always NULL values for the roll and pitch angles,
#                so the rotation is expressed with a 3x3 matrix on the Z-axis
#                with yaw angle "gamma"
# - Translation: it is the initial position of the Drone. It is rotated with
#                the previous matrix.
def create_Matrix_Rotation(client, Drone, D_T_meters, last_gamma,
                           KALMAN_INCLUDE_QUATERNION):
    if KALMAN_INCLUDE_QUATERNION:
        # In this case we don't need to take in consideration the
        # orientation of the drone
        # in the conversion between frames because this will be Kalman
        # Filter concern
        gamma = 0
        last_gamma = 0

    else:
        # Otherwise the K-F doesn't receive updates of the yaw angle from
        # us, so we take it in consideration in the Matrix
        # in order to correctly convert between frames.

        # We get the orientation of the Drone from which we want to get the
        # Yaw angle
        Euler_XYZ = client.GetSegmentGlobalRotationEulerXYZ(Drone, Drone)
        # We are interested only in the first part of the structure:
        Euler_radians = Euler_XYZ[0]

        # In case we receive tha value [0,0,0], that means an error with the
        # Vicon Tracker, we continue using the previuos value of the yaw angle.
        if (Euler_radians[0] == 0 and Euler_radians[1] == 0 and Euler_radians[
            2] == 0):
            gamma = last_gamma
        else:
            gamma = Euler_radians[2]
            last_gamma = gamma

    # roll angle:
    alpha = 0
    # pitch angle:
    beta = 0

    # We build the homogeneous rotation matrix that will convert the
    # positions expressed in the Vicon reference system
    # in positions expressed in the Body frame of the Drone
    R_x = np.array([[1, 0, 0], [0, math.cos(alpha), -math.sin(alpha)],
                    [0, math.sin(alpha), math.cos(alpha)]])
    R_y = np.array([[math.cos(beta), 0, math.sin(beta)], [0, 1, 0],
                    [-math.sin(beta), 0, math.cos(beta)]])
    R_z = np.array([[math.cos(gamma), -math.sin(gamma), 0],
                    [math.sin(gamma), math.cos(gamma), 0], [0, 0, 1]])
    Matrix_Rotation_xy = np.dot(R_x, R_y)
    Matrix_Rotation_xyz = np.dot(Matrix_Rotation_xy, R_z)
    Matrix_Rotation = np.transpose(Matrix_Rotation_xyz)
    Matrix_homogeneous = np.array([[Matrix_Rotation[0][0],
                                    Matrix_Rotation[0][1],
                                    Matrix_Rotation[0][2], -
                                    np.dot(Matrix_Rotation,
                                           np.transpose(D_T_meters))[0]],
                                   [Matrix_Rotation[1][0],
                                    Matrix_Rotation[1][1],
                                    Matrix_Rotation[1][2], -
                                    np.dot(Matrix_Rotation,
                                           np.transpose(D_T_meters))[1]],
                                   [Matrix_Rotation[2][0],
                                    Matrix_Rotation[2][1],
                                    Matrix_Rotation[2][2], -
                                    np.dot(Matrix_Rotation,
                                           np.transpose(D_T_meters))[2]],
                                   [0, 0, 0, 1]])

    # In case we update the last value of the yaw angle (not null value) we
    # update returning it toghether with the matrixes:
    return Matrix_homogeneous, Matrix_Rotation, last_gamma


def create_Matrix_Rotation(client, Drone, D_T_meters, last_gamma,KALMAN_INCLUDE_QUATERNION):

    if KALMAN_INCLUDE_QUATERNION:
        # In this case we don't need to take in consideration the
        # orientation of the drone in the conversion betwwen frames because
        # this will be Kalman Filter concern
        gamma = 0
        last_gamma = 0

    else:
        # Otherwise the K-F doesn't receive updates of the yaw angle from us,
        # so we take it in consideration in the Matrix in order to correctly
        # convert between frames.

        # We get the orientation of the Drone from which we want to get the
        # Yaw angle
        Euler_XYZ = client.GetSegmentGlobalRotationEulerXYZ(Drone, Drone)
        # We are interested only in the first part of the structure:
        Euler_radians = Euler_XYZ[0]

        # In case we receive tha value [0,0,0], that means an error with the
        # Vicon Tracker, we continue using the previuos value of the yaw angle.
        if \
        Euler_radians[0] == 0 and Euler_radians[1] == 0 and Euler_radians[2] == 0:
            gamma = last_gamma
        else:
            gamma = Euler_radians[2]
            last_gamma = gamma

    # roll angle:
    alpha = 0
    # pitch angle:
    beta = 0

    # We build the homogeneous rotation matrix that will convert the
    # positions expressed in the Vicon reference system in positions
    # expressed in the Body frame of the Drone
    R_x = np.array([[1 , 0, 0], [0, math.cos(alpha), -math.sin(alpha)], [0, math.sin(alpha), math.cos(alpha)]])
    R_y = np.array([[math.cos(beta), 0, math.sin(beta)], [0, 1, 0], [-math.sin(beta), 0, math.cos(beta)]])
    R_z = np.array([[math.cos(gamma), -math.sin(gamma), 0], [math.sin(gamma), math.cos(gamma), 0], [0, 0, 1]])
    Matrix_Rotation_xy = np.dot(R_x, R_y)
    Matrix_Rotation_xyz = np.dot(Matrix_Rotation_xy, R_z)
    Matrix_Rotation = np.transpose(Matrix_Rotation_xyz)
    Matrix_homogeneous = np.array([[Matrix_Rotation[0][0], Matrix_Rotation[0][1], Matrix_Rotation[0][2], -np.dot(Matrix_Rotation, np.transpose(D_T_meters))[0]], [Matrix_Rotation[1][0], Matrix_Rotation[1][1], Matrix_Rotation[1][2], -np.dot(Matrix_Rotation, np.transpose(D_T_meters))[1]], [Matrix_Rotation[2][0], Matrix_Rotation[2][1], Matrix_Rotation[2][2], -np.dot(Matrix_Rotation, np.transpose(D_T_meters))[2]], [0, 0, 0, 1]])

    # In case we update the last value of the yaw angle (not null value) we
    # update returning it toghether with the matrix:
    return Matrix_homogeneous,last_gamma


# This function implements the landing phase that starts at the end of the
# experiment
def landing(last_position_send, SUBTRACTED_HEIGHT, DELTA_HEIGHT, cf, lg_stab,
            fdesc, MAKE_LOG_FILE):
    # We gradually reduce the height setpoint
    while last_position_send[2] - SUBTRACTED_HEIGHT > 0:
        cf.commander.send_position_setpoint(
            last_position_send[0],
            last_position_send[1],
            last_position_send[2] - SUBTRACTED_HEIGHT,
            0)
        SUBTRACTED_HEIGHT = SUBTRACTED_HEIGHT + DELTA_HEIGHT

    if MAKE_LOG_FILE:
        # close the connection with the log table:
        lg_stab.stop()
        fdesc.close()

    exit()

    return
