# ------------------------------------COMMENTS---------------------------------
# -----------------------------------------------------------------------------
# -----------------------------------------------------------------------------
"""
NB:
    TESTS
    1) Execute this file to track the Wand for WAIT_TIME seconds
    2) Deletion of the waiting time constraint (WAIT_TIME) and realtime track
        of the Wand with a safety offset
    3) Test Wand turn-off and end of experiment
    4) Add the possibility to start from anywhere in the room, with relative
        conversions between reference systems
    -->5) Add Kalman Filter with relative movements instead of a pursuit
"""
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
import own_module as crazy

# -----------------------------------------------------------------------------
# ----------------------------------SET UP-------------------------------------
# -----------------------------------------------------------------------------

VICON_IP = "192.168.0.2"  # Set the IP of the Vicon server to connect to
VICON_PORT = "801"  # Set the port of the Vicon server to connect to
Drone = "Crazyflie"  # Set the "Vicon name" of the object relative to the drone

# Set the "Vicon-name" of the object relative to the Wand
Wand = "Active Wand v2 (Origin Tracking)"
uri = 'radio://0/80/2M'  # Used for the connection with the drone

# -----------------------------------------------------------------------------
# ----------------------------------VARIABLES----------------------------------
# -----------------------------------------------------------------------------

# Used in the generation of the position reference for the drone
T_prec = np.array([0, 0, 0])
# Used in the generation of the position reference for the drone
W_T_prec = np.array([0, 0, 0])
take_off = 1  # It will be set to 0 after the take off
# It is updated in each iteration and it will be used
# as the starting point for the landing phase.
last_position_send = np.array([0, 0, 0])
# Used to update the yaw angle of the Drone during the experiment
last_gamma = 0
# Current height of the Drone. It is used only in the take off phase.
height_drone = 0

WAIT_TIME = 5  # [s]  Used for the management of the iterations
SLEEP_TIME = 0.001  # [s]  Used for the management of the iterations
ITERATIONS = WAIT_TIME / SLEEP_TIME  # Used to manage the iterations
OFFSET = 0.3  # [m]  Security offset

# TODO: move constants and settings to another file (own_module?)

# Current number of consecutive loss in the acquisition of the wand position
CONSECUTIVE_LOSS = 0
# To be subtracted from the "z" component of the last position of the drone
# during each iteration of the landing phase
DELTA_HEIGHT = 0.01  # [m]
# Sum of meters subtracted from the  "z" component of
# the last position of the drone
SUBTRACTED_HEIGHT = 0.01  # [m]
# [m]  The height the drone has to reach at the end of the take off. This can't
# be higher than the "MOTION_COMMANDER_DEFAULT_HEIGHT" used in the class of the
# Motion Commander. We suggest to set it at least at 90% of its value.
DEFAULT_HEIGHT = 0.5
# [m]  #This is the default height used by the Motion Commander during take
# off. If it has to be higher than DEFAULT_HEIGHT because we consider the take
# off phase concluded once the drone reaches DEFAULT_HEIGHT, but this can't be
# true if they are equal and maybe the Vicon observe a minor value
# (due to noise)
MOTION_COMMANDER_DEFAULT_HEIGHT = 0.8
# Used indicate when the Drone is in the Take off Phase or not.
# TODO: why re-declared?
take_off = 0

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

# Extracted from PyVicon Library:
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--hostname", "--" + VICON_IP + ":" + VICON_PORT)
args = parser.parse_args()

# Create a VICON Client
client = ViconDataStream.Client()

# Connect to the VICON Server running on the same LAN
client.Connect(VICON_IP + ":" + VICON_PORT)
print("Connected")

# Check setting the buffer size works
client.SetBufferSize(1000)
print("Buffer created")

# Enable all the data types
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

# Reference system
client.SetAxisMapping(ViconDataStream.Client.AxisMapping.EForward,
                      ViconDataStream.Client.AxisMapping.ELeft,
                      ViconDataStream.Client.AxisMapping.EUp)
xAxis, yAxis, zAxis = client.GetAxisMapping()
print('X Axis', xAxis, 'Y Axis', yAxis, 'Z Axis', zAxis)

# ------------------------------------------------------------------------------------------------------------------------------------------------------------
# ----------------------------------MAIN
# ----------------------------------------------------------------------------------------------------------------------
# ------------------------------------------------------------------------------------------------------------------------------------------------------------

# we create the log file where we print all the variable of interest:
fdesc = open("datalog_FF_" + datetime.now().strftime("%Y%m%d_%H%M%S") + ".txt",
             "w")

# DRONE CONNECTION
print('CONNECTION...')
logging.basicConfig(level=logging.ERROR)
cflib.crtp.init_drivers(enable_debug_driver=False)
cf = cflib.crazyflie.Crazyflie()

First_Position = crazy.get_First_Position(client, Drone)

# Class used to start the synchronization with the drone:
with SyncCrazyflie(uri, cf) as scf:
    # We prepare and open the connection to address the Log Table:
    lg_stab = crazy.config_logging(scf)
    # !!! Uncomment if we don't use the FlowDeck !!!
    # cf.param.set_value('stabilizer.estimator', '2')
    # reset_estimator(cf)
    lg_stab.start()

    # TAKE OFF
    # Class used for the position control during the take-off phase:
    with MotionCommander(scf, MOTION_COMMANDER_DEFAULT_HEIGHT) as mc:

        print('TAKE OFF')

        while height_drone < DEFAULT_HEIGHT:

            # Get position of the drone in the Vicon reference System:
            a = client.GetFrame()
            b = client.GetFrameNumber()
            D_T_tuple = client.GetSegmentGlobalTranslation(Drone, Drone)
            D_T_millimeters = D_T_tuple[0]
            D_T_meters = np.array([float(D_T_millimeters[0]) / 1000,
                                   float(D_T_millimeters[1]) / 1000,
                                   float(D_T_millimeters[2]) / 1000])
            # Used for store the Drone's position in the Vicon System.
            D_T_vicon = D_T_meters

        # Get orientation of the drone from the Vicon:
        Euler_angles = client.GetSegmentGlobalRotationEulerXYZ(Drone,
                                                               Drone)
        Drone_orientation = Euler_angles[0]

        # We convert vectors in homogenous vector and we convert the
        # position in the body frame:
        Matrix_homogeneous, last_gamma = crazy.create_Matrix_Rotation(
            client,
            Drone,
            First_Position,
            last_gamma)
        D_T_homogenous = np.array(
            [D_T_meters[0], D_T_meters[1], D_T_meters[2], 1])
        D_T_homogenous = np.dot(Matrix_homogeneous,
                                np.transpose(D_T_homogenous))
        D_T_meters = np.array(
            [D_T_homogenous[0], D_T_homogenous[1], D_T_homogenous[2]])

        # We write in the log file with the following format:
        #       drone's position body frame                 quaternions
        #       drone's position Vicon frame                drone's
        #       orientation from Vicon
        #       setpoint body frame             drone's position and
        #       orientation from log table
        print(D_T_meters[0],
              D_T_meters[1],
              D_T_meters[2],
              0,
              0,
              0,
              0,
              D_T_vicon[0],
              D_T_vicon[1],
              D_T_vicon[2],
              Drone_orientation[0],
              Drone_orientation[1],
              Drone_orientation[2],
              0,
              0,
              DEFAULT_HEIGHT,
              0,
              log_pos_x,
              log_pos_y,
              log_pos_z,
              log_roll,
              log_pitch,
              log_yaw,
              file=fdesc)

        # !!! Uncomment if we want to send measures to Kalman filter
        # also during the take off. We suggest to don't do this while we
        # use the motion commander. !!!
        # cf.extpos.send_extpos(D_T_meters[0], D_T_meters[1],
        # D_T_meters[2])

        # Update the current height of the drone:
        height_drone = D_T_meters[2]

    # In case we receive [0,0,0] from vicon (this means that an error
    # occurred with the tracker) we store the last position of the drone
    # for using it instead of the "outlier" [0,0,0]
    last_drone_position = np.array(
        [D_T_meters[0], D_T_meters[1], D_T_meters[2]])

    # this is the end of the take off phase
    take_off = 0

    while 1:

        if take_off == 0:
            try:

                # We get the Wand's position expressed in the Vicon
                # system and we convert it from [mm] to [m]:
                a = client.GetFrame()
                b = client.GetFrameNumber()
                W_T_tuple = client.GetSegmentGlobalTranslation(Wand,
                                                               'Root')
                W_T_millimeters = W_T_tuple[0]
                W_T_meters = np.array([float(W_T_millimeters[0]) / 1000,
                                       float(W_T_millimeters[1]) / 1000,
                                       float(W_T_millimeters[2]) / 1000])

                # if we receive the value [0,0,0] from the vicon means
                # that an error occurred with the Tracker.
                # So if the error persists for more than MAX_LOSS times
                # we stop the experiment, otherwise
                # it is considered as a sort of "outlier" and we use the
                # last correct position used instead of it.
                if (W_T_meters[0] == 0 and W_T_meters[1] == 0 and
                        W_T_meters[2] == 0):

                    CONSECUTIVE_LOSS = CONSECUTIVE_LOSS + 1
                    W_T_meters = last_position_send
                    cf.commander.send_position_setpoint(W_T_meters[0],
                                                        W_T_meters[1],
                                                        W_T_meters[2], 0)

                    if CONSECUTIVE_LOSS == crazy.MAX_LOSS:
                        # LANDING PHASE:
                        # We received MAX_LOSS consecutive "null
                        # position" of the wand so we decide to start
                        # landing.
                        print("START LANDING: ", crazy.MAX_LOSS,
                              " consecutive null position of the Wand "
                              "have been received.")

                        # We gradually reduce the height setpoint:
                        while (last_position_send[
                                   2] - SUBTRACTED_HEIGHT > 0):
                            cf.commander.send_position_setpoint(
                                last_position_send[0],
                                last_position_send[1],
                                last_position_send[2] - SUBTRACTED_HEIGHT,
                                0)
                            SUBTRACTED_HEIGHT = SUBTRACTED_HEIGHT + \
                                DELTA_HEIGHT

                        # close the connection with the log table:
                        lg_stab.stop()
                        fdesc.close()
                        exit()
                else:

                    CONSECUTIVE_LOSS = 0

                    Matrix_homogeneous, last_gamma = \
                        crazy.create_Matrix_Rotation(
                            client, Drone, First_Position,
                            last_gamma)

                    # We pass from a Wand's position expressed in the
                    # Vicon frame to a Wand's position expressed
                    # in the Body frame of the Drone before sending it
                    # as the new setpoint:
                    a = client.GetFrame()
                    b = client.GetFrameNumber()
                    W_T_homogenous = np.array(
                        [W_T_meters[0] + OFFSET, W_T_meters[1] + OFFSET,
                         W_T_meters[2], 1])
                    W_T_homogenous = np.dot(Matrix_homogeneous,
                                            np.transpose(W_T_homogenous))
                    W_T_meters = np.array(
                        [W_T_homogenous[0], W_T_homogenous[1],
                         W_T_homogenous[2]])

                    # We send the new setpoint:
                    cf.commander.send_position_setpoint(W_T_meters[0],
                                                        W_T_meters[1],
                                                        W_T_meters[2], 0)
                    last_position_send = W_T_meters

                    # We will use this information when we write in the
                    # log file:
                    Euler_angles = client.GetSegmentGlobalRotationEulerXYZ(
                        Drone, Drone)
                    Drone_orientation = Euler_angles[0]

                    # We get the actual position of Drone expressed in
                    # the Vicon frame:
                    D_T_tuple = client.GetSegmentGlobalTranslation(Drone,
                                                                   Drone)
                    D_T_millimeters = D_T_tuple[0]
                    D_T_meters = np.array(
                        [float(D_T_millimeters[0]) / 1000,
                         float(D_T_millimeters[1]) / 1000,
                         float(D_T_millimeters[2]) / 1000])
                    D_T_vicon = D_T_meters

                    # if it is equal to [0,0,0], it meas that en error
                    # from vicon occurred so we use the last correct
                    # position instead of it.
                    if (D_T_meters[0] == 0 and D_T_meters[1] == 0 and
                            D_T_meters[2] == 0):
                        D_T_meters = last_drone_position
                    # otherwise we convert the position in the body
                    # frame before sending it for the update of the
                    # Kalman filter:
                    else:
                        D_T_homogenous = np.array(
                            [D_T_meters[0], D_T_meters[1], D_T_meters[2],
                             1])
                        D_T_homogenous = np.dot(Matrix_homogeneous,
                                                np.transpose(
                                                    D_T_homogenous))
                        D_T_meters = np.array(
                            [D_T_homogenous[0], D_T_homogenous[1],
                             D_T_homogenous[2]])

                        # update the last correct value for the drone
                        # position:
                        last_drone_position = D_T_meters

                    # Update the Kalman filter sending the last measure
                    # for the drone position:
                    cf.extpos.send_extpos(D_T_meters[0], D_T_meters[1],
                                          D_T_meters[2])
                    # print("KALMAN UPDATE: ", D_T_meters[0],
                    # D_T_meters[1], D_T_meters[2])

                    # We write in the log file with the following format:
                    #       drone's position body frame
                    #       quaternions     drone's position vicon frame
                    #       drone's orientation from Vicon
                    #       setpoint body frame
                    #       drone's position and orientation from log table
                    print(D_T_meters[0], D_T_meters[1], D_T_meters[2], 0,
                          0, 0, 0, D_T_vicon[0], D_T_vicon[1],
                          D_T_vicon[2], Drone_orientation[0],
                          Drone_orientation[1], Drone_orientation[2],
                          W_T_meters[0], W_T_meters[1], W_T_meters[2], 0,
                          log_pos_x, log_pos_y, log_pos_z, log_roll,
                          log_pitch, log_yaw, file=fdesc)

            # In case something wrong happens, we manage the exception
            # with the start of the landing procedure:
            except ViconDataStream.DataStreamException as e:
                print('START LANDING: This error form Vicon occurred: \n',
                      e)

                # LANDING PHASE:
                # We gradually reduce the height of the setpoint:
                while last_position_send[2] - SUBTRACTED_HEIGHT > 0:
                    cf.commander.send_position_setpoint(
                        last_position_send[0], last_position_send[1],
                        last_position_send[2] - SUBTRACTED_HEIGHT, 0)
                    SUBTRACTED_HEIGHT = SUBTRACTED_HEIGHT + DELTA_HEIGHT

                # We close the connection with the log table:
                lg_stab.stop()
                fdesc.close()
                exit()
