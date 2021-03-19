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
import time

# -----------------------------------------------------------------------------
# ----------------------------------SET UP-------------------------------------
# -----------------------------------------------------------------------------

VICON_IP = "192.168.0.2"  # Set the IP of the Vicon server to connect to
VICON_PORT = "801"  # Set the port of the Vicon server to connect to
drone = "Crazyflie"  # Set the "Vicon name" of the object relative to the drone

# Set the "Vicon-name" of the object relative to the Wand
Wand = "Active Wand v2 (Origin Tracking)"
uri = 'radio://0/80/2M'  # Used for the connection with the drone

# Set to 0 if you don't want to obtain the log file after
# the execution. Otherwise set to 1.
# The log file can be used by the MATLAB file "plot.m"
MAKE_LOG_FILE = 0

# Set to 1 if you want to send orientation together with the setpoint to
# the Kalman filter.
KALMAN_wQUATERNION = 0

# We strictly recommend not to send Kalman updates during take-off
# if you use the MotionCommander (so keep it set to zero)
KALMAN_inTAKEOFF = 0

# Set to 1 if you want to do a test without making the drone fly.
# In that case we suggest to set the other flags to these values:
#   LOG_TEST_THRUSTER_DEACTIVATED = 1 (Obviously)
#   MAKE_LOG_FILE = 1
#   KALMAN_wQUATERNION = 1
#   KALMAN_inTAKEOFF = 1
TEST_noTHRUSTER = 0

# -----------------------------------------------------------------------------
# ----------------------------------VARIABLES----------------------------------
# -----------------------------------------------------------------------------
# TODO: move variables to module
# Used in the generation of the setpoint reference for the drone
last_trans = np.array([0, 0, 0])
# Used in the generation of the setpoint reference for the drone
last_wand_trans = np.array([0, 0, 0])

take_off = 1  # It will be set to 0 after the take off

# This is updated in each iteration and it will be used
# as the starting point for the landing phase.
last_sent_pos = np.array([0, 0, 0])

# Used to update the yaw angle of the drone during the experiment
last_gamma = 0

# Current height of the drone. It is used only in the take-off phase.
drone_height = 0

# Used to store the orientation of the drone
quaternion = np.array([0, 0, 0, 0])
# Used in the updating of the orientation of the drone to the Kalman filter
last_quaternion = np.array([0, 0, 0, 0])

# Security offset
SEC_OFFSET = 0.3  # [m]

# Current number of consecutive losses in the acquisition of the Wand setpoint
CONSEC_LOSSES = 0

# To be subtracted from the "z" component of the last setpoint of the drone
# during each iteration of the landing phase
# TODO: why? cannot use/trust sensors?
DELTA_HEIGHT = 0.01  # [m]

# Sum of meters subtracted from the  "z" component of
# the last setpoint of the drone
# TODO: useless?
SUBTRACTED_HEIGHT = 0.01  # [m]

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
# ----------------------------------VICON CONNECTION---------------------------
# -----------------------------------------------------------------------------

# Extracted from PyVicon Library
print("Connecting with the Vicon tracker...")

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--hostname", "--" + VICON_IP + ":" + VICON_PORT)
args = parser.parse_args()

# Create a VICON Client
vicon = ViconDataStream.Client()

# Connect to the VICON Server running on the same LAN
vicon.Connect(VICON_IP + ":" + VICON_PORT)
print("Connected to Vicon!")

# Setting a buffer size to work with
vicon.SetBufferSize(frame_num)
print("Buffer of ", str(frame_num), " frames created.")

# Enable all the data types (action needed to be able to use these)
vicon.EnableSegmentData()
vicon.EnableMarkerData()
vicon.EnableUnlabeledMarkerData()
vicon.EnableMarkerRayData()
vicon.EnableDeviceData()
vicon.EnableCentroidData()
print("Data types enabled.")

# Report whether the data types have been enabled
print('Segments: ', vicon.IsSegmentDataEnabled())
print('Markers: ', vicon.IsMarkerDataEnabled())
print('Unlabeled Markers: ', vicon.IsUnlabeledMarkerDataEnabled())
print('Marker Rays: ', vicon.IsMarkerRayDataEnabled())
print('Devices: ', vicon.IsDeviceDataEnabled())
print('Centroids: ', vicon.IsCentroidDataEnabled())

#           Trying to set different stream modes
# "ClientPull": Increases latency, network bandwidth kept at minimum,
# buffers unlikely to be filled up
vicon.SetStreamMode(ViconDataStream.Client.StreamMode.EClientPull)
print('Getting a data frame in ClientPull mode: ', vicon.GetFrame(),
      "; pulled frame number: ", vicon.GetFrameNumber())
# "ClientPreFetch": improved ClientPull, server performances unlikely to be
# affected, latency slightly reduced, buffers unlikely to be filled up
vicon.SetStreamMode(ViconDataStream.Client.StreamMode.EClientPullPreFetch)
print('Getting a data frame in PreFetch mode', vicon.GetFrame(),
      "; pulled frame number: ", vicon.GetFrameNumber())
# "ServerPush": the servers pushes frames to the client, best for latency,
# frames dropped only if all buffers are full
vicon.SetStreamMode(ViconDataStream.Client.StreamMode.EServerPush)
print('Getting a data frame in ServerPush mode: ', vicon.GetFrame(),
      "; pulled frame number: ", vicon.GetFrameNumber())

print("ServerPush mode set.")

# Show the frame rate from both client and server side
print('Current frame rate: ', vicon.GetFrameRate(), " Hz.")
print('Frame rates: ')
for frameRateName, frameRateValue in vicon.GetFrameRates().items():
    print(frameRateName, ": ", frameRateValue, " Hz")

# Setting reference system
vicon.SetAxisMapping(ViconDataStream.Client.AxisMapping.EForward,
                     ViconDataStream.Client.AxisMapping.ELeft,
                     ViconDataStream.Client.AxisMapping.EUp)
xAxis, yAxis, zAxis = vicon.GetAxisMapping()
print('X Axis: ', xAxis)
print('Y Axis: ', yAxis)
print('Z Axis: ', zAxis)

# -----------------------------------------------------------------------------
# ----------------------------------FLYING MODE--------------------------------
# -----------------------------------------------------------------------------

# DRONE CONNECTION
print('Connecting with the drone...')

if MAKE_LOG_FILE:
    # we create a log file in which we print all the variable of interest
    # TODO: make a better log file.
    filename = "LogFile_CrazyFlie_FOLLOWING" \
               + datetime.now().strftime("%Y%m%d_%H%M%S")
    if TEST_noTHRUSTER:
        filename = filename + "_ThrustOFF_"
    else:
        filename = filename + "_ThrustON_"
    if KALMAN_inTAKEOFF:
        filename = filename + "_KFTakeoff_"
    else:
        filename = filename + "_NoKFTakeoff_"
    if KALMAN_wQUATERNION:
        filename = filename + "KFwQuat"
    else:
        filename = filename + "KFwoQuat"

    # Only logs of level ERROR or above will be tracked
    # https://docs.python.org/3/library/logging.html#levels
    logging.basicConfig(level=logging.ERROR)

    file_desc = open(filename + ".txt", "w")

# Initialize all the drivers
cflib.crtp.init_drivers(enable_debug_driver=False)

# Creating an instance of the Crazyflie object and getting the initial setpoint
cf = cflib.crazyflie.Crazyflie()
first_pos = crazy.getFirstPosition(vicon, drone)

# Class used to start the synchronization with the drone
with SyncCrazyflie(uri, cf) as scf:  # automatic connection

    if MAKE_LOG_FILE:
        # We prepare and open the connection to address the Log Table:
        log_stabilizer = crazy.config_logging(scf)
        log_stabilizer.start()

    # We reset the Kalman Filter before flying
    crazy.reset_estimator(cf)

    # TODO: develop two separated files?
    if TEST_noTHRUSTER == 0:  # Thrusters activated

        # AUTO TAKE-OFF!
        # Class used for the setpoint control during the take-off phase:
        # take-off automatic when context created using "with"
        with MotionCommander(scf, MC_HEIGHT) as mc:
            mc.forward(0.3)
            print("forward done")
            mc.back(0.3)
            print("back done")
            time.sleep(5)

            print('Take-Off!')

            while drone_height < DEFAULT_HEIGHT:

                # Request a new data frame from the server and its ordinal
                # number
                data_frame = vicon.GetFrame()
                frame_num = vicon.GetFrameNumber()

                # Get the drone setpoint in the Vicon reference system and
                # convert to meters
                drone_trans = vicon.GetSegmentGlobalTranslation(drone, drone)
                drone_trans_mm = drone_trans[0]
                drone_trans_m = np.array([float(drone_trans_mm[0]) / 1000,
                                          float(drone_trans_mm[1]) / 1000,
                                          float(drone_trans_mm[2]) / 1000])

                # Used to store the drone setpoint in the Vicon System.
                # TODO: is it even useful?
                drone_trans_vicon = drone_trans_m

                # Rotation, in Vicon reference system
                quat_XYZW = vicon. \
                    GetSegmentGlobalRotationQuaternion(drone, drone)
                quaternion = quat_XYZW[0]

                if MAKE_LOG_FILE:
                    # Just for log purposes
                    Euler_angles = vicon. \
                        GetSegmentGlobalRotationEulerXYZ(drone, drone)
                    Drone_orientation = Euler_angles[0]

                # We convert vectors in homogenous matrices and we convert the
                # setpoint in the body frame
                # TODO: check with "GetSegmentGlobalRotationMatrix" result
                hom_mat, last_gamma = \
                    crazy.createMatrixRotation(vicon, drone, first_pos,
                                               last_gamma, KALMAN_wQUATERNION)
                # Homogeneous vector containing drone setpoint in Vicon
                # reference system
                drone_trans_hom = np.array([drone_trans_m[0],
                                            drone_trans_m[1],
                                            drone_trans_m[2],
                                            1])
                # Conversion in drone body system
                drone_trans_hom = np.dot(hom_mat,
                                         np.transpose(drone_trans_hom))
                # Selection of the first three components (not homogeneous
                # vector anymore)
                drone_trans_m = np.array([drone_trans_hom[0],
                                          drone_trans_hom[1],
                                          drone_trans_hom[2]])

                if MAKE_LOG_FILE:
                    # We write in the log file with the following format:
                    # drone setpoint body frame
                    # quaternions
                    # drone setpoint Vicon frame
                    # drone orientation from Vicon
                    # setpoint body frame
                    # drone setpoint and orientation from log table
                    # noinspection PyUnboundLocalVariable
                    print(drone_trans_m[0], drone_trans_m[1], drone_trans_m[2],
                          quaternion[0], quaternion[1], quaternion[2],
                          quaternion[3],
                          drone_trans_vicon[0], drone_trans_vicon[1],
                          drone_trans_vicon[2],
                          Drone_orientation[0], Drone_orientation[1],
                          Drone_orientation[2],
                          0, 0, DEFAULT_HEIGHT, 0,
                          crazy.log_pos_x, crazy.log_pos_y, crazy.log_pos_z,
                          crazy.log_roll, crazy.log_pitch, crazy.log_yaw,
                          file=file_desc)

                if KALMAN_inTAKEOFF:
                    # Send measures to Kalman filter during take-off.
                    # We suggest not to do this while you use the
                    # MotionCommander class.
                    if KALMAN_wQUATERNION:
                        cf.extpos.send_extpose(drone_trans_m[0],
                                               drone_trans_m[1],
                                               drone_trans_m[2],
                                               quaternion[0],
                                               quaternion[1],
                                               quaternion[2],
                                               quaternion[3])
                    else:
                        cf.extpos.send_extpos(drone_trans_m[0],
                                              drone_trans_m[1],
                                              drone_trans_m[2])

                # Update the current height of the drone:
                drone_height = drone_trans_m[2]

            # Reminder: out of "while drone_height < DEFAULT_HEIGHT"

            # Store the last setpoint of the drone
            last_drone_position = np.array([drone_trans_m[0],
                                            drone_trans_m[1],
                                            drone_trans_m[2]])

            # This is the end of the take off phase
            take_off = 0

            while 1:

                # TODO: cleaner way?
                if take_off == 0:
                    try:

                        # Get the Wand setpoint expressed in the Vicon
                        # system and we convert it to meters
                        data_frame = vicon.GetFrame()
                        frame_num = vicon.GetFrameNumber()
                        wand_trans = vicon. \
                            GetSegmentGlobalTranslation(Wand, 'Root')

                        wand_trans_mm = wand_trans[0]
                        wand_trans_m = np.array(
                            [float(wand_trans_mm[0]) / 1000,
                             float(wand_trans_mm[1]) / 1000,
                             float(wand_trans_mm[2]) / 1000])

                        # If the error (0,0,0) persists for more than
                        # crazy.MAX_LOSS times, we stop the experiment;
                        # otherwise it is considered as a sort of "outlier"
                        # and we use the last correct setpoint instead of it.
                        # The same also happens when we decide to stop the
                        # experiment turning off the wand.
                        if (wand_trans_m[0] == 0
                                and wand_trans_m[1] == 0
                                and wand_trans_m[2] == 0):

                            CONSEC_LOSSES += 1
                            wand_trans_m = last_sent_pos
                            cf.commander.send_position_setpoint(
                                wand_trans_m[0],
                                wand_trans_m[1],
                                wand_trans_m[2],
                                0)

                            if CONSEC_LOSSES == crazy.MAX_LOSS:
                                # LANDING

                                print("Start landing: ", crazy.MAX_LOSS,
                                      " consecutive null setpoint of the Wand "
                                      "have been received.")
                                # TODO: global log config and variables?
                                crazy.landing(last_sent_pos,
                                              SUBTRACTED_HEIGHT, DELTA_HEIGHT,
                                              cf, log_stabilizer, file_desc,
                                              MAKE_LOG_FILE)
                        else:

                            CONSECUTIVE_LOSS = 0

                            if KALMAN_wQUATERNION:
                                # In this case we don't have to compute
                                # again the matrix for the rotation because
                                # it is constant.
                                # The drone knows its current value of the
                                # yaw from KF and we don't have to change
                                # the conversion of the coordinates.
                                quaternion_XYZW = \
                                    vicon.GetSegmentGlobalRotationQuaternion(
                                        drone,
                                        drone)
                                quaternion = quaternion_XYZW[0]

                                # If we receive from the Vicon all
                                # quaternions set to 0; we sent the last
                                # correct value
                                if (quaternion[0] == 0
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
                                hom_mat, last_gamma = crazy. \
                                    createMatrixRotation(vicon, drone,
                                                         first_pos, last_gamma,
                                                         KALMAN_wQUATERNION)

                            # We pass from a Wand's setpoint expressed in the
                            # Vicon frame to a Wand's setpoint expressed
                            # in the Body frame of the drone before sending it
                            # as the new setpoint:
                            data_frame = vicon.GetFrame()
                            frame_num = vicon.GetFrameNumber()
                            wand_trans_hom = np.array(
                                [wand_trans_m[0] + SEC_OFFSET,
                                 wand_trans_m[1] + SEC_OFFSET,
                                 wand_trans_m[2],
                                 1])
                            wand_trans_hom = np. \
                                dot(hom_mat, np.transpose(wand_trans_hom))
                            wand_trans_m = np.array([wand_trans_hom[0],
                                                     wand_trans_hom[1],
                                                     wand_trans_hom[2]])

                            # We send the new setpoint
                            cf.commander.send_position_setpoint(
                                wand_trans_m[0],
                                wand_trans_m[1],
                                wand_trans_m[2],
                                0)

                            last_sent_pos = wand_trans_m

                            if MAKE_LOG_FILE:
                                # We will use this information when we write
                                # in the log file:
                                Euler_angles = \
                                    vicon.GetSegmentGlobalRotationEulerXYZ(
                                        drone, drone)
                                Drone_orientation = Euler_angles[0]

                            # We get the actual setpoint of drone expressend
                            # in the Vicon frame:
                            drone_trans = vicon.GetSegmentGlobalTranslation(
                                drone, drone)
                            drone_trans_mm = drone_trans[0]
                            drone_trans_m = np.array([
                                float(drone_trans_mm[0]) / 1000,
                                float(drone_trans_mm[1]) / 1000,
                                float(drone_trans_mm[2]) / 1000
                            ])
                            drone_trans_vicon = drone_trans_m

                            # if it is equal to [0,0,0], it meas that en
                            # error from vicon occurred so we use the last
                            # correct setpoint instead of it.
                            if drone_trans_m[0] == 0 \
                                    and drone_trans_m[1] == 0 \
                                    and drone_trans_m[2] == 0:
                                drone_trans_m = last_drone_position
                            # otherwise we convert the setpoint in the body
                            # frame before sending it for the update of the
                            # Kalman filter:
                            else:
                                drone_trans_hom = np.array([
                                    drone_trans_m[0],
                                    drone_trans_m[1],
                                    drone_trans_m[2],
                                    1])
                                drone_trans_hom = np.dot(
                                    hom_mat,
                                    np.transpose(drone_trans_hom))
                                drone_trans_m = np.array([
                                    drone_trans_hom[0],
                                    drone_trans_hom[1],
                                    drone_trans_hom[2]])

                                # update the last correct value for the
                                # drone setpoint:
                                last_drone_position = drone_trans_m

                            if KALMAN_wQUATERNION:
                                # Update the Kalman filter sending the last
                                # measure of both the drone setpoint and
                                # orientation:
                                cf.extpos.send_extpose(drone_trans_m[0],
                                                       drone_trans_m[1],
                                                       drone_trans_m[2],
                                                       quaternion[0],
                                                       quaternion[1],
                                                       quaternion[2],
                                                       quaternion[3])
                            else:
                                # Update the Kalman filter sending the last
                                # measure of the drone setpoint
                                cf.extpos.send_extpos(drone_trans_m[0],
                                                      drone_trans_m[1],
                                                      drone_trans_m[2])

                            if MAKE_LOG_FILE:
                                # We write in the log file with the
                                # following format:
                                #       drone's setpoint body frame
                                #       quaternions
                                #       drone's setpoint Vicon frame
                                #       drone's orientation from Vicon
                                #       setpoint body frame
                                #       drone's setpoint and orientation
                                #       from log table
                                print(drone_trans_m[0], drone_trans_m[1],
                                      drone_trans_m[2],
                                      quaternion[0], quaternion[1],
                                      quaternion[2], quaternion[3],
                                      drone_trans_vicon[0],
                                      drone_trans_vicon[1],
                                      drone_trans_vicon[2],
                                      Drone_orientation[0],
                                      Drone_orientation[1],
                                      Drone_orientation[2],
                                      wand_trans_m[0], wand_trans_m[1],
                                      wand_trans_m[2],
                                      0,
                                      crazy.log_pos_x, crazy.log_pos_y,
                                      crazy.log_pos_z,
                                      crazy.log_roll, crazy.log_pitch,
                                      crazy.log_yaw,
                                      file=file_desc)

                    # In case something wrong happens, we manage the
                    # exception with the start of the landing procedure:
                    except ViconDataStream.DataStreamException as exc:
                        print("START LANDING: This error was reported",
                              "from Vicon: ", '\n', exc)
                        crazy.landing(last_sent_pos, SUBTRACTED_HEIGHT,
                                      DELTA_HEIGHT, cf, log_stabilizer,
                                      file_desc, MAKE_LOG_FILE)

    # -------------------------------------------------------------------------
    # ----------------------------------DISABLED THRUSTER MODE-----------------
    # -------------------------------------------------------------------------

    else:
        # Here we have the condition TEST_noTHRUSTER == 1
        print("Starting Test mode with thrusters deactivated...")

        try:
            while 1:

                # Get setpoint of the drone in the Vicon reference System
                data_frame = vicon.GetFrame()
                frame_num = vicon.GetFrameNumber()

                drone_trans = vicon.GetSegmentGlobalTranslation(drone, drone)
                drone_trans_mm = drone_trans[0]
                drone_trans_m = np.array([float(drone_trans_mm[0]) / 1000,
                                          float(drone_trans_mm[1]) / 1000,
                                          float(drone_trans_mm[2]) / 1000])

                # Used for store the drone's setpoint in the Vicon System.
                drone_trans_vicon = drone_trans_m

                if MAKE_LOG_FILE:
                    # Get orientation of the drone from the Vicon:
                    Euler_angles = vicon.GetSegmentGlobalRotationEulerXYZ(
                        drone, drone)
                    Drone_orientation = Euler_angles[0]

                # We convert vectors in homogenous vector and we convert the
                # setpoint in the body frame:
                hom_mat, last_gamma = crazy. \
                    createMatrixRotation(vicon, drone, first_pos, last_gamma,
                                         KALMAN_wQUATERNION)
                drone_trans_hom = np.array([drone_trans_m[0],
                                            drone_trans_m[1],
                                            drone_trans_m[2],
                                            1])
                drone_trans_hom = np.dot(hom_mat,
                                         np.transpose(drone_trans_hom))
                drone_trans_m = np.array([drone_trans_hom[0],
                                          drone_trans_hom[1],
                                          drone_trans_hom[2]])

                quat_XYZW = vicon.GetSegmentGlobalRotationQuaternion(
                    drone, drone)
                quaternion = quat_XYZW[0]

                if MAKE_LOG_FILE:
                    # We write in the log file with the following format:
                    #       drone setpoint (body frame)
                    #       quaternions
                    #       drone setpoint (Vicon frame)
                    #       drone orientation from Vicon
                    #       setpoint body frame
                    #       drone setpoint
                    #       orientation from log table
                    # noinspection PyUnboundLocalVariable
                    print(drone_trans_m[0], drone_trans_m[1], drone_trans_m[2],
                          quaternion[0], quaternion[1], quaternion[2],
                          quaternion[3],
                          drone_trans_vicon[0], drone_trans_vicon[1],
                          drone_trans_vicon[2],
                          Drone_orientation[0], Drone_orientation[1],
                          Drone_orientation[2],
                          0, 0, DEFAULT_HEIGHT, 0,
                          crazy.log_pos_x, crazy.log_pos_y, crazy.log_pos_z,
                          crazy.log_roll, crazy.log_pitch, crazy.log_yaw,
                          file=file_desc)

                if KALMAN_inTAKEOFF:
                    # send measures to Kalman filter also during the take
                    # off. We suggest to don't do this while you use the
                    # motion commander.
                    if KALMAN_wQUATERNION:
                        cf.extpos.send_extpose(drone_trans_m[0],
                                               drone_trans_m[1],
                                               drone_trans_m[2],
                                               quaternion[0], quaternion[1],
                                               quaternion[2], quaternion[3])
                    else:
                        cf.extpos.send_extpos(drone_trans_m[0],
                                              drone_trans_m[1],
                                              drone_trans_m[2])

        # In case something wrong happens, we manage the exception with the
        # start of the landing procedure:
        except ViconDataStream.DataStreamException as exc:
            print('Stopping Test! \n', "Vicon reported: ", exc)

            if MAKE_LOG_FILE:
                # close the connection with the log table:
                log_stabilizer.stop()
                file_desc.close()

            exit()
