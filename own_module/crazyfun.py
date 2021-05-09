# Module containing all the function used in the Crazyflie project

from __future__ import print_function
import __main__
import logging
import os
import math
import numpy as np
import time
from datetime import datetime, timedelta
from timeit import default_timer as timer
from pathlib import Path
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.crazyflie.log import LogConfig
from vicon_dssdk import ViconDataStream
import script_variables as sc_v
import script_setup as sc_s

# TODO: complete functions help


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
    log_pos_y = -data['stateEstimate.y']
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


def datetime2matlabdatenum(dt):
    mdn = dt + timedelta(days=366)
    frac_seconds = (dt-datetime(dt.year, dt.month, dt.day, 0, 0, 0)).seconds /\
                   (24.0 * 60.0 * 60.0)
    frac_microseconds = dt.microsecond / (24.0 * 60.0 * 60.0 * 1000000.0)
    return mdn.toordinal() + frac_seconds + frac_microseconds


def print_callback(timestamp, data, log_conf):
    """
    Prints gathered data to a specific file.

        :param data: Data to be logged
        :param timestamp
        :param log_conf
        :return:None

    """

    pos_x = data['stateEstimate.x']
    pos_y = data['stateEstimate.y']
    pos_z = data['stateEstimate.z']
    roll = data['stabilizer.roll']
    pitch = data['stabilizer.pitch']
    yaw = data['stabilizer.yaw']

    int_matlab.write(pos_x, pos_y, pos_z, roll, pitch, yaw)


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
    # TODO: Add more to log:
    # kalman stddev, posi/or stddev, extQuatStdDev 0.1, extPosStdDev

    data_log_async(sync_crazyflie, measure_log)

    return measure_log


def reset_estimator(scf):
    cf = scf.cf
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)

    wait_for_position_estimator(cf)


def wait_for_position_estimator(scf):
    logging.info('Waiting for estimator to find setpoint...')

    log_config = LogConfig(name='Kalman Pos Variance', period_in_ms=500)
    log_config.add_variable('kalman.varPX', 'float')
    log_config.add_variable('kalman.varPY', 'float')
    log_config.add_variable('kalman.varPZ', 'float')

    log_config2 = LogConfig(name='Kalman Or Variance', period_in_ms=500)
    log_config2.add_variable('kalman.q0', 'float')
    log_config2.add_variable('kalman.q1', 'float')
    log_config2.add_variable('kalman.q2', 'float')
    log_config2.add_variable('kalman.q3', 'float')

    var_x_history = [1000] * 10
    var_y_history = [1000] * 10
    var_z_history = [1000] * 10
    var_q0_history = [1000] * 10
    var_q1_history = [1000] * 10
    var_q2_history = [1000] * 10
    var_q3_history = [1000] * 10

    threshold = 0.001

    with SyncLogger(scf, log_config) as logger, \
            SyncLogger(scf, log_config2) as logger2:
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

            if (max_x - min_x) < threshold and \
                    (max_y - min_y) < threshold and \
                    (max_z - min_z) < threshold:
                break

        # for log_entry in logger2:
        #     data = log_entry[1]
        #
        #     var_q0_history.append(data['kalman.q0'])
        #     var_q0_history.pop(0)
        #     var_q1_history.append(data['kalman.q1'])
        #     var_q1_history.pop(0)
        #     var_q2_history.append(data['kalman.q2'])
        #     var_q2_history.pop(0)
        #     var_q3_history.append(data['kalman.q3'])
        #     var_q3_history.pop(0)
        #
        #     min_q0 = min(var_q0_history)
        #     max_q0 = max(var_q0_history)
        #     min_q1 = min(var_q1_history)
        #     max_q1 = max(var_q1_history)
        #     min_q2 = min(var_q2_history)
        #     max_q2 = max(var_q2_history)
        #     min_q3 = min(var_q3_history)
        #     max_q3 = max(var_q3_history)
        #
        #     logging.debug("dq0: %s dq1: %s dq2: %s dq3: %s",
        #                   max_q0 - min_q0, max_q1 - min_q1, max_q2 - min_q2,
        #                   max_q3 - min_q3)
        #
        #     if (max_q0 - min_q0) < threshold and \
        #             (max_q1 - min_q1) < threshold and \
        #             (max_q2 - min_q2) < threshold and \
        #             (max_q3 - min_q3) < threshold:
        #         break


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


def quat_product(q0, q1):
    """both Vicon and Crazy-Kalman use the form (x, y, z, w),
    NOT the common (w, x, y, z)
    """

    x0, y0, z0, w0 = q0
    x1, y1, z1, w1 = q1

    q0q1_w = w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1
    q0q1_x = w0 * x1 + x0 * w1 + y0 * z1 - z0 * y1
    q0q1_y = w0 * y1 - x0 * z1 + y0 * w1 + z0 * x1
    q0q1_z = w0 * z1 + x0 * y1 - y0 * x1 + z0 * w1

    return np.array((q0q1_x, q0q1_y, q0q1_z, q0q1_w))


def rot_with_quat(vector, quat):
    """
    quat: (qx, qy, qz, qw)
    """
    q_conj = np.array((-quat[0], -quat[1], -quat[2], quat[3]))
    v = np.array((vector[0], vector[1], vector[2], 0))
    return quat_product(quat_product(quat, v), q_conj)


def quaternion_rotation_matrix(quaternion):
    """
    Covert a quaternion into a full three-dimensional rotation matrix.

    Input
    :param quaternion: A 4 element array representing the quaternion;
                    normally would be (qw,qx,qy,qz), Vicon uses (qx,qy,qz, qw)

    Output
    :return: A 3x3 element matrix representing the full 3D rotation matrix.
             This rotation matrix converts a point in the local reference
             frame to a point in the global reference frame.
    """
    # Reformat Vicon quaternion
    quat = (quaternion[3], quaternion[0], quaternion[1], quaternion[2])
    # Extract the values from quat
    qw = quat[0]
    qx = quat[1]
    qy = quat[2]
    qz = quat[3]

    # First row of the rotation matrix
    r00 = 2 * (qw * qw + qx * qx) - 1
    r01 = 2 * (qx * qy - qw * qz)
    r02 = 2 * (qx * qz + qw * qy)

    # Second row of the rotation matrix
    r10 = 2 * (qx * qy + qw * qz)
    r11 = 2 * (qw * qw + qy * qy) - 1
    r12 = 2 * (qy * qz - qw * qx)

    # Third row of the rotation matrix
    r20 = 2 * (qx * qz - qw * qy)
    r21 = 2 * (qy * qz + qw * qx)
    r22 = 2 * (qw * qw + qz * qz) - 1

    # 3x3 rotation matrix
    rot_matrix = np.array([[r00, r01, r02],
                           [r10, r11, r12],
                           [r20, r21, r22]])

    return rot_matrix


def rotz(angle):
    angle_rad = angle*math.pi/180
    mat = np.mat([[math.cos(angle_rad), -math.sin(angle_rad), 0],
                 [math.sin(angle_rad), math.cos(angle_rad), 0],
                 [0, 0, 1]])
    return mat


def rotx(angle):
    angle_rad = angle*math.pi/180
    mat = np.mat([[1, 0, 0],
                 [0, math.cos(angle_rad), -math.sin(angle_rad)],
                 [0, math.sin(angle_rad), math.cos(angle_rad)]])
    return mat


def roty(angle):
    angle_rad = angle*math.pi/180
    mat = np.mat([[math.cos(angle_rad), 0, math.sin(angle_rad)],
                  [0, 1, 0],
                  [-math.sin(angle_rad), 0, math.cos(angle_rad)]])
    return mat


def hom_mat(rot, pos):
    pos_col = np.array(pos, ndmin=2).T
    mat = np.concatenate(
        (np.concatenate((rot, pos_col), axis=1),
         np.array([0, 0, 0, 1], ndmin=2)), axis=0)
    return mat


def to_drone_global(position_v, orientation_v, g_pos_v, g_or_v):
    """
    Converts pose into Drone initial frame.

    :param position_v:
    :type position_v:
    :param orientation_v:
    :type orientation_v:
    :param g_pos_v:
    :type g_pos_v:
    :param g_or_v:
    :type g_or_v:
    :return:
    :rtype:
    """

    quat_v2g = np.array((-g_or_v[0], -g_or_v[1],
                         -g_or_v[2], g_or_v[3]))
    quat_v2g_conj = np.array(g_or_v)

    point_pos_v = np.array((position_v[0], position_v[1], position_v[2], 0))
    g_pos_v = np.array((g_pos_v[0], g_pos_v[1], g_pos_v[2], 0))

    point_pos_g = quat_product(quat_v2g,
                               quat_product(point_pos_v - g_pos_v,
                                            quat_v2g_conj))

    setpoint_g = point_pos_g[:3]
    # setpoint_g = np.array((float(point_pos_g[0] / 1000),
    #                        float(point_pos_g[1] / 1000),
    #                        float(point_pos_g[2] / 1000)))
    final_yaw = quat_product(quat_v2g, orientation_v)

    return setpoint_g, final_yaw


def to_drone_relative(position_v, orientation_v, g_pos_v, g_or_v):
    """
    Converts pose into Drone initial frame.

    :param position_v:
    :type position_v:
    :param orientation_v:
    :type orientation_v:
    :param g_pos_v:
    :type g_pos_v:
    :param g_or_v:
    :type g_or_v:
    :return:
    :rtype:
    """
    quat_v2g = np.array((-g_or_v[0], -g_or_v[1],
                         -g_or_v[2], g_or_v[3]))
    quat_v2g_conj = np.array(g_or_v)

    point_pos_v = np.array((position_v[0], position_v[1], position_v[2], 0))
    g_pos_v = np.array((g_pos_v[0], g_pos_v[1], g_pos_v[2], 0))

    point_pos_g = quat_product(quat_v2g,
                               quat_product(point_pos_v,
                                            quat_v2g_conj))

    setpoint_g = np.array((float(point_pos_g[0] / 1000),
                           float(point_pos_g[1] / 1000),
                           float(point_pos_g[2] / 1000)))
    final_yaw = quat_product(quat_v2g, orientation_v)

    return setpoint_g, final_yaw


def yaw2quat(yaw):
    # input yaw in degree
    yaw = yaw*math.pi/180
    qx, qy = 0, 0
    qz = math.sin(yaw/2)
    qw = math.cos(yaw/2)
    return np.array((qx, qy, qz, qw))


def quat2yaw(q):
    yaw = 2 * math.acos(q[3])
    return yaw * 180 / math.pi  # degree


def repeat_fun(period, func, *args):
    def time_tick():
        t = time.time()
        while True:
            t += period
            yield max(t - time.time(), 0)

    tick = time_tick()
    while True:
        time.sleep(next(tick))
        func(*args)


def pose_sending(sync_cf):
    start = timer()
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

    sync_cf.cf.extpos.send_extpose(sc_v.drone_pos[0], sc_v.drone_pos[1],
                                   sc_v.drone_pos[2],
                                   sc_v.drone_or[0], sc_v.drone_or[1],
                                   sc_v.drone_or[2], sc_v.drone_or[3])

    logging.debug("sent pose: ", str(sc_v.drone_pos), str(sc_v.drone_or))

    vicon_matlab.write(sc_v.drone_pos[0], sc_v.drone_pos[1],
                       sc_v.drone_pos[2],
                       sc_v.drone_or[0], sc_v.drone_or[1],
                       sc_v.drone_or[2], sc_v.drone_or[2])

    end = timer()
    logging.debug("elapsed time: ", end-start)


class MatlabPrint:
    """
    flag = 0 -> points obtained from Vicon
    flag = 1 -> setpoints sent to the drone
    flag = 2 -> drone internal estimation
    """
    descriptor = 0

    def __init__(self, flag=0):
        file_name = os.path.normpath(__main__.__file__).split(os.sep)[-1][:-3]

        if flag == 2:
            mat_file = "../internal_data/" + file_name \
                       + datetime.now().strftime("__%Y%m%d_%H%M%S")
        elif flag == 1:
            mat_file = "../setpoint_data/" + file_name \
                       + datetime.now().strftime("__%Y%m%d_%H%M%S")
        else:  # flag == 0
            mat_file = "../vicon_data/" + file_name \
                       + datetime.now().strftime("__%Y%m%d_%H%M%S")
        mat_file = mat_file + ".txt"
        ff = os.path.normpath(
            os.path.join(Path(__file__).parent.absolute(),
                         mat_file))

        self.descriptor = open(ff, 'a')

    def __del__(self):
        if self.descriptor:
            self.descriptor.close()

    def __enter__(self):
        return self

    def __exit__(self):
        if self.descriptor:
            self.descriptor.close()

    def write(self, *args):
        # Create a string with all data passed to the function.
        # You can use a \n as argument to get a newline
        s = ""
        for arg in args:
            s = s + "{} ".format(str(arg))

        # timestamp to use in MATLAB
        s = s + " {}".format(str(datetime2matlabdatenum(datetime.now())))
        print(s, file=self.descriptor)


log_pos_x = 0
log_pos_y = 0
log_pos_z = 0
log_roll = 0
log_pitch = 0
log_yaw = 0

vicon_matlab = MatlabPrint(flag=0)
set_matlab = MatlabPrint(flag=1)
int_matlab = MatlabPrint(flag=2)
