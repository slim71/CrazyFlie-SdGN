# -----------------------------------------------------------------------------
# ----------------------------------IMPORT-------------------------------------
# -----------------------------------------------------------------------------
from __future__ import print_function
from vicon_dssdk import ViconDataStream
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.crtp.crtpstack import CRTPPacket
from datetime import datetime
import numpy as np
import cflib.utils
import argparse
import logging
from own_module import crazyfun as crazy

# -----------------------------------------------------------------------------
# ----------------------------------SET UP-------------------------------------
# -----------------------------------------------------------------------------

VICON_IP = "192.168.0.2"  # Set the IP of the Vicon server to connect to
VICON_PORT = "801"  # Set the port of the Vicon server to connect to
Drone = "Crazyflie"  # Set the "Vicon name" of the object relative to the drone

# Set the "Vicon-name" of the object relative to the Wand
Wand = "Active Wand v2 (Origin Tracking)"
uri = 'radio://0/80/2M'  # Used for the connection with the drone

# Set to 0 if you don't want to obtain the log file after
# the execution. Otherwise set to 1.
# The log file can be used by the MATLAB file "plot.m"
MAKE_LOG_FILE = 0

# Set to 1 if you want to send orientation together with the position to
# the Kalman filter.
KALMAN_INCLUDE_QUATERNION = 0

# We strictly recommend not to send Kalman updates during take-off
# if you use the MotionCommander (so keep it set to zero)
ACTIVATE_KALMAN_DURING_TAKEOFF = 0

# Set to 1 if you want to do a test without making the Drone fly.
# In that case we suggest to set the other flags to these values:
#   LOG_TEST_THRUSTER_DEACTIVATED = 1 (Obviously)
#   MAKE_LOG_FILE = 1
#   KALMAN_INCLUDE_QUATERNION = 1
#   ACTIVATE_KALMAN_DURING_TAKEOFF = 1
LOG_TEST_WITH_DEACTIVATED_THRUSTER = 0

# -----------------------------------------------------------------------------
# ----------------------------------VARIABLES----------------------------------
# -----------------------------------------------------------------------------

# Used in the generation of the position reference for the drone
T_prec = np.array([0, 0, 0])
# Used in the generation of the position reference for the drone
W_T_prec = np.array([0, 0, 0])

take_off = 1  # It will be set to 0 after the take off

# This is updated in each iteration and it will be used
# as the starting point for the landing phase.
last_position_sent = np.array([0, 0, 0])

# Used to update the yaw angle of the Drone during the experiment
last_gamma = 0

# Current height of the Drone. It is used only in the take-off phase.
height_drone = 0

# Used to store the orientation of the drone
quaternion = np.array([0, 0, 0, 0])
# Used in the updating of the orientation of the drone to the Kalman filter
last_quaternion = np.array([0, 0, 0, 0])

# Security offset
OFFSET = 0.3  # [m]

# Max number of consecutive loss allowed during acquisition
# of the position of the Wand or Drone
MAX_LOSS = 10

# Current number of consecutive loss in the acquisition of the wand position
CONSECUTIVE_LOSS = 0

# To be subtracted from the "z" component of the last position of the drone
# during each iteration of the landing phase
# TODO: why? cannot use/trust sensors?
DELTA_HEIGHT = 0.01  # [m]

# Sum of meters subtracted from the  "z" component of
# the last position of the droned
# TODO: useless?
SUBTRACTED_HEIGHT = 0.01  # [m]

# The height the drone has to reach at the end of the take-off. This can't
# be higher than the "MOTION_COMMANDER_DEFAULT_HEIGHT" used in the class of the
# Motion Commander. We suggest to set it at least at 90% of its value.
DEFAULT_HEIGHT = 0.5  # [m]

# This is the default height used by the Motion Commander during take-off.
# It has to be higher than DEFAULT_HEIGHT because we consider the
# take-off phase concluded once the drone reaches DEFAULT_HEIGHT, but this
# can't be always true because the Vicon might observe another value due to
# noise
MOTION_COMMANDER_DEFAULT_HEIGHT = 0.8  # [m]

# Variables used to store the parameters of the Drone's log table (TOC)
log_pos_x = 0.0
log_pos_y = 0.0
log_pos_z = 0.0
log_roll = 0.0
log_pitch = 0.0
log_yaw = 0.0

# -----------------------------------------------------------------------------
# ----------------------------------VICON CONNECTION---------------------------
# -----------------------------------------------------------------------------

# Extracted from PyVicon Library
print("CONNECTING WITH VICON TRACKER...")

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--hostname", "--" + VICON_IP + ":" + VICON_PORT)
args = parser.parse_args()

# Create a VICON Client
client = ViconDataStream.Client()

# Connect to the VICON Server running on the same LAN
client.Connect(VICON_IP + ":" + VICON_PORT)
print("Connected")

# Setting a buffer size to work with
client.SetBufferSize(1000)
print("Buffer created")

# Enable all the data types TODO: why needed?
client.EnableSegmentData()
client.EnableMarkerData()
client.EnableUnlabeledMarkerData()
client.EnableMarkerRayData()
client.EnableDeviceData()
client.EnableCentroidData()
print("Data types enabled")

# Report whether the data types have been enabled
print('Segments', client.IsSegmentDataEnabled())
print('Markers', client.IsMarkerDataEnabled())
print('Unlabeled Markers', client.IsUnlabeledMarkerDataEnabled())
print('Marker Rays', client.IsMarkerRayDataEnabled())
print('Devices', client.IsDeviceDataEnabled())
print('Centroids', client.IsCentroidDataEnabled())

# Try setting the different stream modes
# TODO: why needed? Which is used and which is the best?
client.SetStreamMode(ViconDataStream.Client.StreamMode.EClientPull)
print('Get Frame Pull', client.GetFrame(), client.GetFrameNumber())
client.SetStreamMode(ViconDataStream.Client.StreamMode.EClientPullPreFetch)
print('Get Frame PreFetch', client.GetFrame(), client.GetFrameNumber())
client.SetStreamMode(ViconDataStream.Client.StreamMode.EServerPush)
print('Get Frame Push', client.GetFrame(), client.GetFrameNumber())
print('Frame Rate', client.GetFrameRate())

# Show the frame rate from both client and server side
print('Frame Rates')
for frameRateName, frameRateValue in client.GetFrameRates().items():
    print(frameRateName, frameRateValue)

# Setting reference system
client.SetAxisMapping(ViconDataStream.Client.AxisMapping.EForward,
                      ViconDataStream.Client.AxisMapping.ELeft,
                      ViconDataStream.Client.AxisMapping.EUp)
xAxis, yAxis, zAxis = client.GetAxisMapping()
print('X Axis', xAxis, 'Y Axis', yAxis, 'Z Axis', zAxis)

# -----------------------------------------------------------------------------
# ----------------------------------FLYING MODE--------------------------------
# -----------------------------------------------------------------------------

# DRONE CONNECTION
print('CONNECTING WITH DRONE...')

if MAKE_LOG_FILE:
    # we create a log file in which we print all the variable of interest
    # TODO: make a better log file.
    filename = "Log_File_CrazyFlie_RELATIVE" + datetime.now().strftime(
        "%Y%m%d_%H%M%S")
    if LOG_TEST_WITH_DEACTIVATED_THRUSTER:
        filename = filename + "_ThrusterOff_"
    else:
        filename = filename + "_ThrusterOn_"
    if ACTIVATE_KALMAN_DURING_TAKEOFF:
        filename = filename + "_KalmanDuringTakeOff_"
    else:
        filename = filename + "_NoKalmanTakeOff_"
    if KALMAN_INCLUDE_QUATERNION:
        filename = filename + "KalmanWithQuaternion"
    else:
        filename = filename + "KalmanWithoutQuaternion"

    fdesc = open(filename + ".txt", "w")

# TODO: to move in the if above, since it's for logging purposes?
logging.basicConfig(level=logging.ERROR)
cflib.crtp.init_drivers(enable_debug_driver=False)

# Creating an instance of the Crazyflie object and getting the initial position
cf = cflib.crazyflie.Crazyflie()
First_Position = crazy.getFirstPosition(client, Drone)

# Class used to start the synchronization with the drone
with SyncCrazyflie(uri, cf) as scf:  # automatic connection
    # We prepare and open the connection to address the Log Table
    # TODO: isn't this just for logging purposes? if so, move beneath
    lg_stab = crazy.config_logging(scf)

    if MAKE_LOG_FILE:
        # We prepare and open the connection to address the Log Table:
        lg_stab.start()

    # We reset the Kalman Filter before flying
    crazy.reset_estimator(cf)

    # TODO: isn't it easier to develop this as the "normal" case?
    if LOG_TEST_WITH_DEACTIVATED_THRUSTER == 0:  # Thrusters activated

        # AUTO TAKE-OFF!
        # Class used for the position control during the take-off phase:
        # take-off automatic when context created using "with"
        with MotionCommander(scf, MOTION_COMMANDER_DEFAULT_HEIGHT) as mc:

            print('TAKE OFF')

            while height_drone < DEFAULT_HEIGHT:

                # Get frame and then Drone position of the drone in the Vicon
                # reference system
                # TODO: a,b not used, so why needed?
                a = client.GetFrame()
                b = client.GetFrameNumber()

                D_T_tuple = client.GetSegmentGlobalTranslation(Drone, Drone)
                D_T_millimeters = D_T_tuple[0]
                D_T_meters = np.array([float(D_T_millimeters[0]) / 1000,
                                       float(D_T_millimeters[1]) / 1000,
                                       float(D_T_millimeters[2]) / 1000])

                # Used to store the Drone position in the Vicon System.
                D_T_vicon = D_T_meters

                # TODO: only if orientation used; why both?
                # Rotation, in Vicon ref system
                quaternion_XYZW = client. \
                    GetSegmentGlobalRotationQuaternion(Drone, Drone)
                quaternion = quaternion_XYZW[0]
                Euler_angles = client. \
                    GetSegmentGlobalRotationEulerXYZ(Drone, Drone)
                Drone_orientation = Euler_angles[0]

                # We convert vectors in homogenous matrices and we convert the
                # position in the body frame
                # TODO: matrix_Rotation not needed as a returned value?
                # TODO: it exists "GetSegmentGlobalRotationMatrix"
                Matrix_homogeneous, Matrix_Rotation, last_gamma = \
                    crazy.createMatrixRotation(client, Drone, First_Position,
                                               last_gamma,
                                               KALMAN_INCLUDE_QUATERNION)
                # Homogeneous vector containing Drone position in Vicon
                # reference system
                D_T_homogenous = np.array([D_T_meters[0],
                                           D_T_meters[1],
                                           D_T_meters[2],
                                           1])
                # Conversion in Drone body system
                D_T_homogenous = np.dot(Matrix_homogeneous,
                                        np.transpose(D_T_homogenous))
                # Selection of the first three components (not homogeneous
                # vector anymore)
                D_T_meters = np.array([D_T_homogenous[0],
                                       D_T_homogenous[1],
                                       D_T_homogenous[2]])

                if MAKE_LOG_FILE:
                    # We write in the log file with the following format:
                    # drone position body frame
                    # quaternions
                    # drone position Vicon frame
                    # drone orientation from Vicon
                    # setpoint body frame
                    # drone position and orientation from log table
                    print(D_T_meters[0], D_T_meters[1], D_T_meters[2],
                          quaternion[0], quaternion[1], quaternion[2],
                          quaternion[3], D_T_vicon[0], D_T_vicon[1],
                          D_T_vicon[2], Drone_orientation[0],
                          Drone_orientation[1], Drone_orientation[2], 0, 0,
                          DEFAULT_HEIGHT, 0, log_pos_x, log_pos_y, log_pos_z,
                          log_roll, log_pitch, log_yaw, file=fdesc)

                if ACTIVATE_KALMAN_DURING_TAKEOFF:
                    # Send measures to Kalman filter during the take-off.
                    # We suggest not to do this while you use the
                    # MotionCommander class.
                    if KALMAN_INCLUDE_QUATERNION:
                        cf.extpos.send_extpose(D_T_meters[0], D_T_meters[1],
                                               D_T_meters[2], quaternion[0],
                                               quaternion[1], quaternion[2],
                                               quaternion[3])
                    else:
                        cf.extpos.send_extpos(D_T_meters[0], D_T_meters[1],
                                              D_T_meters[2])

                # Update the current height of the drone:
                height_drone = D_T_meters[2]

            # Reminder: out of "while height_drone < DEFAULT_HEIGHT"

            # Store the last position of the drone
            last_drone_position = np.array([D_T_meters[0],
                                            D_T_meters[1],
                                            D_T_meters[2]])

            # We keep separated the variables dedicated to the Kalman
            # update (last_drone_position) and the setpoint update
            # (last_drone_setpoint)
            last_drone_setpoint = np.array([D_T_meters[0],
                                            D_T_meters[1],
                                            D_T_meters[2]])

            # This is the end of the take off phase
            take_off = 0

            while 1:

                # TODO: cleaner way?
                if take_off == 0:
                    try:

                        # We get the Wand position expressed in the Vicon
                        # system and we convert it from [mm] to [m]:
                        a = client.GetFrame()
                        b = client.GetFrameNumber()
                        W_T_tuple = client. \
                            GetSegmentGlobalTranslation(Wand, 'Root')

                        W_T_millimeters = W_T_tuple[0]
                        W_T_meters = np.array(
                            [float(W_T_millimeters[0]) / 1000,
                             float(W_T_millimeters[1]) / 1000,
                             float(W_T_millimeters[2]) / 1000])

                        # If the error (0,0,0) persists for more than MAX_LOSS
                        # times, we stop the experiment; otherwise
                        # it is considered as a sort of "outlier" and we use
                        # the last correct position instead of it.
                        # The same also happens when we decide to stop the
                        # experiment turning off the wand.
                        if (W_T_meters[0] == 0
                                and W_T_meters[1] == 0
                                and W_T_meters[2] == 0):

                            CONSECUTIVE_LOSS = CONSECUTIVE_LOSS + 1
                            W_T_meters = last_position_sent
                            cf.commander.send_position_setpoint(
                                last_drone_setpoint[0],
                                last_drone_setpoint[1],
                                last_drone_setpoint[2],
                                0)

                            if CONSECUTIVE_LOSS == MAX_LOSS:
                                # LANDING

                                print("START LANDING: ", MAX_LOSS,
                                      " consecutive null position of the Wand "
                                      "have been received.")
                                crazy.landing(last_drone_setpoint,
                                              SUBTRACTED_HEIGHT, DELTA_HEIGHT,
                                              cf, lg_stab, fdesc,
                                              MAKE_LOG_FILE)
                        else:

                            if FIRST_ITERATION or CONSECUTIVE_LOSS:
                                # At the first iteration the translation has
                                # to be a null vector because there isn't a
                                # early value.
                                # Also in case of >=1 losses it has to be zero.

                                Wand_Translation = np.array([0, 0, 0])
                                W_T_prec = np.array([W_T_meters[0],
                                                     W_T_meters[1],
                                                     W_T_meters[2]])

                            else:
                                # We compute the translation of the wand:
                                Wand_Translation[0] = \
                                    W_T_meters[0] - W_T_prec[0]
                                Wand_Translation[1] = \
                                    W_T_meters[1] - W_T_prec[1]
                                Wand_Translation[2] = \
                                    W_T_meters[2] - W_T_prec[2]

                                # We update the value to use in the next
                                # translation:
                                W_T_prec = np.array([W_T_meters[0],
                                                     W_T_meters[1],
                                                     W_T_meters[2]])

                            CONSECUTIVE_LOSS = 0

                            if FIRST_ITERATION == 0:
                                # We convert the translation of the Wand in
                                # body frame in order to use it to compute
                                # the next setpoint for the drone
                                Wand_Translation = \
                                    np.dot(Matrix_Rotation,
                                           np.transpose(Wand_Translation))

                            # Compute the new setpoint
                            last_drone_setpoint[0] = \
                                last_drone_setpoint[0] + Wand_Translation[0]
                            last_drone_setpoint[1] = \
                                last_drone_setpoint[1] + Wand_Translation[1]
                            last_drone_setpoint[2] = \
                                last_drone_setpoint[2] + Wand_Translation[2]

                            # Notifying the end of a (perhaps) first iteration
                            FIRST_ITERATION = 0

                            # We get the actual position of Drone expressed
                            # in the Vicon frame:
                            D_T_tuple = client. \
                                GetSegmentGlobalTranslation(Drone, Drone)
                            D_T_millimeters = D_T_tuple[0]
                            D_T_meters = np.array(
                                [float(D_T_millimeters[0]) / 1000,
                                 float(D_T_millimeters[1]) / 1000,
                                 float(D_T_millimeters[2]) / 1000])
                            D_T_vicon = D_T_meters

                            # if it is equal to [0,0,0], it meas that en
                            # error from Vicon occurred so we use the last
                            # correct position instead of it.
                            if (D_T_meters[0] == 0 and D_T_meters[1] == 0 and
                                    D_T_meters[2] == 0):
                                D_T_meters = last_drone_position
                            else:
                                # otherwise we convert the position in the body
                                # frame before sending it for the update of the
                                # Kalman filter:
                                D_T_homogenous = np.array([D_T_meters[0],
                                                           D_T_meters[1],
                                                           D_T_meters[2],
                                                           1])
                                D_T_homogenous = np. \
                                    dot(Matrix_homogeneous,
                                        np.transpose(D_T_homogenous))
                                D_T_meters = np.array([D_T_homogenous[0],
                                                       D_T_homogenous[1],
                                                       D_T_homogenous[2]])

                                # update the last correct value for the
                                # drone position:
                                last_drone_position = D_T_meters

                            # receive from the Vicon all quaternions
                            quaternion_XYZW = client. \
                                GetSegmentGlobalRotationQuaternion(Drone,
                                                                   Drone)
                            quaternion = quaternion_XYZW[0]

                            if KALMAN_INCLUDE_QUATERNION:
                                # In this case we don't have to compute
                                # again the matrix for the rotation because
                                # it is constant.
                                # The drone knows its current value of the
                                # yaw from KF and we don't have to change
                                # the conversion of the coordinates.

                                # if we receive from the Vicon all
                                # quaternions set to 0; we sent the last
                                # correct value
                                if (
                                        quaternion[0] == 0
                                        and quaternion[1] == 0
                                        and quaternion[2] == 0
                                        and quaternion[3] == 0):

                                    quaternion = last_quaternion

                                else:
                                    last_quaternion = quaternion

                            else:
                                # If we don't send the orientation to the
                                # KF, we have to compute the matrix in each
                                # iteration because
                                # the drone doesn't know its current yaw and
                                # we have to rotate in function of the
                                # current yaw angle
                                Matrix_homogeneous, Matrix_Rotation, \
                                last_gamma = crazy.createMatrixRotation(client,
                                                                        Drone,
                                                                        First_Position,
                                                                        last_gamma,
                                                                        KALMAN_INCLUDE_QUATERNION)

                            # We send the new setpoint
                            cf.commander.send_position_setpoint(
                                last_drone_setpoint[0], last_drone_setpoint[1],
                                last_drone_setpoint[2], 0)

                            # We will use this information when we write in
                            # the log file:
                            Euler_angles = \
                                client.GetSegmentGlobalRotationEulerXYZ(
                                    Drone, Drone)
                            Drone_orientation = Euler_angles[0]

                            if KALMAN_INCLUDE_QUATERNION:
                                # Update the Kalman filter sending the last
                                # measure of both the drone position and
                                # orientation:
                                cf.extpos.send_extpose(D_T_meters[0],
                                                       D_T_meters[1],
                                                       D_T_meters[2],
                                                       quaternion[0],
                                                       quaternion[1],
                                                       quaternion[2],
                                                       quaternion[3])
                            else:
                                # Update the Kalman filter sending the last
                                # measure of the drone position
                                cf.extpos.send_extpos(D_T_meters[0],
                                                      D_T_meters[1],
                                                      D_T_meters[2])

                            if MAKE_LOG_FILE:
                                # We write in the log file with the
                                # following format:
                                #       drone's position body frame
                                #       quaternions
                                #       drone's position Vicon frame
                                #       drone's orientation from Vicon
                                #       setpoint body frame
                                #       drone's position and orientation
                                #       from log table
                                print(D_T_meters[0], D_T_meters[1],
                                      D_T_meters[2], quaternion[0],
                                      quaternion[1], quaternion[2],
                                      quaternion[3], D_T_vicon[0],
                                      D_T_vicon[1], D_T_vicon[2],
                                      Drone_orientation[0],
                                      Drone_orientation[1],
                                      Drone_orientation[2], W_T_meters[0],
                                      W_T_meters[1], W_T_meters[2], 0,
                                      log_pos_x, log_pos_y, log_pos_z,
                                      log_roll, log_pitch, log_yaw, file=fdesc)

                    # In case something wrong happens, we manage the
                    # exception with the start of the landing procedure:
                    except ViconDataStream.DataStreamException as e:
                        print(
                            'START LANDING: This error form Vicon occurred: '
                            '\n',
                            e)
                        crazy.landing(last_position_sent, SUBTRACTED_HEIGHT,
                                      DELTA_HEIGHT, cf, lg_stab, fdesc,
                                      MAKE_LOG_FILE)

    # -------------------------------------------------------------------------
    # ----------------------------------DISABLED THRUSTER MODE-----------------
    # -------------------------------------------------------------------------

    else:
        # Here we have the condition LOG_TEST_WITH_DEACTIVATED_THRUSTER == 1
        print("Starting Test mode with thruster deactivated...")
        try:
            while 1:

                # Get position of the drone in the Vicon reference System
                a = client.GetFrame()
                b = client.GetFrameNumber()

                D_T_tuple = client.GetSegmentGlobalTranslation(Drone, Drone)
                D_T_millimeters = D_T_tuple[0]
                D_T_meters = np.array([float(D_T_millimeters[0]) / 1000,
                                       float(D_T_millimeters[1]) / 1000,
                                       float(D_T_millimeters[2]) / 1000])

                # Used for store the Drone's position in the Vicon System.
                D_T_vicon = D_T_meters

                if MAKE_LOG_FILE:
                    # Get orientation of the drone from the Vicon:
                    Euler_angles = client.GetSegmentGlobalRotationEulerXYZ(
                        Drone, Drone)
                    Drone_orientation = Euler_angles[0]

                # We convert vectors in homogenous vector and we convert the
                # position in the body frame:
                Matrix_homogeneous, last_gamma = crazy. \
                    createMatrixRotation(client, Drone, First_Position,
                                         last_gamma, KALMAN_INCLUDE_QUATERNION)
                D_T_homogenous = np.array([D_T_meters[0],
                                           D_T_meters[1],
                                           D_T_meters[2],
                                           1])
                D_T_homogenous = np.dot(Matrix_homogeneous,
                                        np.transpose(D_T_homogenous))
                D_T_meters = np.array([D_T_homogenous[0],
                                       D_T_homogenous[1],
                                       D_T_homogenous[2]])

                if KALMAN_INCLUDE_QUATERNION:
                    quaternion_XYZW = \
                        client.GetSegmentGlobalRotationQuaternion(
                            Drone, Drone)
                    quaternion = quaternion_XYZW[0]

                if MAKE_LOG_FILE:
                    # We write in the log file with the following format:
                    #       drone's position body frame
                    #       quaternions
                    #       drone's position vicon frame
                    #       drone's orientation from Vicon
                    #       setpoint body frame             drone's position
                    #       and orientation from log table
                    print(D_T_meters[0], D_T_meters[1], D_T_meters[2],
                          quaternion[0], quaternion[1], quaternion[2],
                          quaternion[3], D_T_vicon[0], D_T_vicon[1],
                          D_T_vicon[2], Drone_orientation[0],
                          Drone_orientation[1], Drone_orientation[2], 0, 0,
                          DEFAULT_HEIGHT, 0, log_pos_x, log_pos_y, log_pos_z,
                          log_roll, log_pitch, log_yaw, file=fdesc)

                if ACTIVATE_KALMAN_DURING_TAKEOFF:
                    # send measures to Kalman filter also during the take
                    # off. We suggest to don't do this while you use the
                    # motion commander.
                    if KALMAN_INCLUDE_QUATERNION:
                        cf.extpos.send_extpose(D_T_meters[0], D_T_meters[1],
                                               D_T_meters[2], quaternion[0],
                                               quaternion[1], quaternion[2],
                                               quaternion[3])
                    else:
                        cf.extpos.send_extpos(D_T_meters[0], D_T_meters[1],
                                              D_T_meters[2])

        # In case something wrong happens, we manage the exception with the
        # start of the landing procedure:
        except ViconDataStream.DataStreamException as e:
            print('STOP TEST: This error form Vicon occurred: \n', e)

            if MAKE_LOG_FILE:
                # close the connection with the log table:
                lg_stab.stop()
                fdesc.close()

            exit()
