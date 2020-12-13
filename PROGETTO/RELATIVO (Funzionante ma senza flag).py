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
drone = "Crazyflie"  # Set the "Vicon name" of the object relative to the drone

# Set the "Vicon-name" of the object relative to the Wand
Wand = "Active Wand v2 (Origin Tracking)"
uri = 'radio://0/80/2M'  # Used for the connection with the drone

# -----------------------------------------------------------------------------
# ----------------------------------VARIABLES----------------------------------
# -----------------------------------------------------------------------------

# Used in the generation of the position reference for the drone
last_trans = np.array([0, 0, 0])
# Used in the generation of the position reference for the drone
last_wand_trans = np.array([0, 0, 0])

take_off = 1  # It will be set to 0 after the take off

# Used to update the yaw angle of the drone during the experiment
last_gamma = 0

WAIT_TIME = 5  # [s]  Used for the management of the iterations
SLEEP_TIME = 0.001  # [s]  Used for the management of the iterations
ITERATIONS = WAIT_TIME / SLEEP_TIME  # Used to manage the iterations

# Security offset
SEC_OFFSET = 0.3  # [m]

# Current number of consecutive losses in the acquisition of the Wand position
CONSEC_LOSSES = 0

# To be subtracted from the "z" component of the last position of the drone
# during each iteration of the landing phase
# TODO: why? cannot use/trust sensors?
DELTA_HEIGHT = 0.01  # [m]

# Sum of meters subtracted from the  "z" component of
# the last position of the drone
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
# ----------------------------------MAIN---------------------------------------
# -----------------------------------------------------------------------------

# DRONE CONNECTION
print('Connecting with the drone...')

logging.basicConfig(level=logging.ERROR)

# Initialize all the drivers
cflib.crtp.init_drivers(enable_debug_driver=False)

# Creating an instance of the Crazyflie object and getting the initial position
cf = cflib.crazyflie.Crazyflie()
first_pos = crazy.getFirstPosition(vicon, drone)

# Class used to start the synchronization with the drone
with SyncCrazyflie(uri, cf) as scf:  # automatic connection

    # AUTO TAKE-OFF!
    # Class used for the position control during the take-off phase:
    # take-off automatic when context created using "with"
    with MotionCommander(scf, default_height=0.55) as mc:

        print('Take-Off!')

        height_drone = 0

        # Before "proper" take-off
        while height_drone < 0.5:

            # Request a new data frame from the server and its ordinal
            # number
            data_frame = vicon.GetFrame()
            frame_num = vicon.GetFrameNumber()

            # Get the drone position in the Vicon reference system and
            # convert to meters
            drone_trans = vicon.GetSegmentGlobalTranslation(drone, drone)
            drone_trans_mm = drone_trans[0]
            drone_trans_m = np.array([float(drone_trans_mm[0]) / 1000,
                                      float(drone_trans_mm[1]) / 1000,
                                      float(drone_trans_mm[2]) / 1000])
            height_drone = drone_trans_m[2]

        last_drone_position = np.array([0, 0, height_drone])
        last_drone_reference = np.array([0, 0, height_drone])
        print(last_drone_reference)
        sdf = 0

        while 1:

            # take_off currently set to 0
            if take_off == 0:  # take off already done
                try:
                    if i == 1:
                        print("Wand position: ", wand_trans_m)  # TODO: ???

                    # Get the Wand position expressed in the Vicon
                    # system and we convert it to meters
                    data_frame = vicon.GetFrame()
                    frame_num = vicon.GetFrameNumber()

                    Matrix_homogeneous, Matrix_Rotation, last_gamma = \
                        crazy.createMatrixRotation(
                            vicon,
                            drone,
                            First_Position,
                            last_gamma)
                    W_T_tuple = vicon.GetSegmentGlobalTranslation(Wand,
                                                                   'Root')
                    # Absolute position of the drone in [mm]<
                    # We are interested only in the first part of the data
                    # structure
                    W_T_millimeters = W_T_tuple[0]
                    W_T_meters = np.array([float(W_T_millimeters[0]) / 1000,
                                           float(W_T_millimeters[1]) / 1000,
                                           float(W_T_millimeters[
                                                     2]) / 1000])  # Pass
                    # from [mm] to [m]
                    # print("New Wand position: ", wand_trans_m)
                    # print("GET FRAME: ",
                    #       wand_trans_m[0],
                    #       wand_trans_m[1],
                    #       wand_trans_m[2])

                    if (W_T_meters[0] == 0 and W_T_meters[1] == 0 and
                            W_T_meters[2] == 0):

                        CONSECUTIVE_LOSS += 1
                        # x, y, z, yaw
                        cf.commander.send_position_setpoint(
                            last_drone_reference[0], last_drone_reference[1],
                            last_drone_reference[2], 0)

                        if CONSECUTIVE_LOSS == crazy.MAX_LOSS:
                            # LANDING PHASE: We received MAX_LOSS consecutive
                            # "null position" of the wand, so We decide to
                            # start landing.
                            # TODO: land() with this
                            print("START LANDING: ", crazy.MAX_LOSS,
                                  " consecutive null position of the Wand have"
                                  " been received.")
                            while (last_drone_reference[
                                       2] - SUBTRACTED_HEIGHT > 0):
                                cf.commander.send_position_setpoint(
                                    last_drone_reference[0],
                                    last_drone_reference[1],
                                    last_drone_reference[
                                        2] - SUBTRACTED_HEIGHT, 0)
                                SUBTRACTED_HEIGHT = SUBTRACTED_HEIGHT + \
                                    DELTA_HEIGHT
                            exit()
                    else:
                        # We pass from a Wand's position expressed in the
                        # Vicon frame to a Wand's position expressed in the
                        # Body frame of the drone before sending it
                        if i == 0 or CONSECUTIVE_LOSS:

                            Wand_Translation = np.array([0, 0, 0])
                            W_T_prec = np.array(
                                [W_T_meters[0], W_T_meters[1], W_T_meters[2]])
                            CONSECUTIVE_LOSS = 0

                        else:
                            CONSECUTIVE_LOSS = 0
                            Wand_Translation[0] = W_T_meters[0] - W_T_prec[0]
                            Wand_Translation[1] = W_T_meters[1] - W_T_prec[1]
                            Wand_Translation[2] = W_T_meters[2] - W_T_prec[2]
                            W_T_prec = np.array(
                                [W_T_meters[0], W_T_meters[1], W_T_meters[2]])

                        if i == 1:
                            Wand_Translation = np.dot(
                                Matrix_Rotation,
                                np.transpose(Wand_Translation)
                            )
                            print("Body translation: ", Wand_Translation)
                        # print("drone reference: ", last_drone_reference)
                        last_drone_reference[0] = last_drone_reference[0] + \
                            Wand_Translation[0]
                        last_drone_reference[1] = last_drone_reference[1] + \
                            Wand_Translation[1]
                        last_drone_reference[2] = last_drone_reference[2] + \
                            Wand_Translation[2]
                        print("New drone reference: ",
                              last_drone_reference, "\n\n")
                        i = 1
                        # We send the actual position of drone expressed in
                        # the Body frame to the Kalman Filter
                        drone_trans = vicon.GetSegmentGlobalTranslation(drone,
                                                                        drone)
                        drone_trans_mm = drone_trans[0]
                        drone_trans_m = np.array(
                            [float(drone_trans_mm[0]) / 1000,
                             float(drone_trans_mm[1]) / 1000,
                             float(drone_trans_mm[2]) / 1000])
                        if (drone_trans_m[0] == 0 and drone_trans_m[1] == 0 and
                                drone_trans_m[2] == 0):
                            drone_trans_m = last_drone_position
                        else:
                            D_T_homogenous = np.array(
                                [drone_trans_m[0], drone_trans_m[1], drone_trans_m[2],
                                 1])
                            D_T_homogenous = np.dot(Matrix_homogeneous,
                                                    np.transpose(
                                                        D_T_homogenous))
                            drone_trans_m = np.array(
                                [D_T_homogenous[0], D_T_homogenous[1],
                                 D_T_homogenous[2]])
                            last_drone_position = drone_trans_m

                        cf.extpos.send_extpos(drone_trans_m[0], drone_trans_m[1],
                                              drone_trans_m[2])
                        print("Location of the drone: ", drone_trans_m[0],
                              drone_trans_m[1], drone_trans_m[2])
                        # We send the new setpoint
                        cf.commander.send_position_setpoint(
                            last_drone_reference[0], last_drone_reference[1],
                            last_drone_reference[2], 0)
                        sdf = sdf + 0.05
                        # print("POSITION SEND: ",
                        #       last_drone_reference[0],
                        #       last_drone_reference[1],
                        #       last_drone_reference[2])

                except ViconDataStream.DataStreamException as e:
                    print('START LANDING: This error from Vicon occurred: \n',
                          e)
                    # LANDING PHASE: We received MAX_LOSS consecutive
                    # "null position" of the wand so we decide to start
                    # landing.
                    while last_drone_reference[2] - SUBTRACTED_HEIGHT > 0:
                        cf.commander.send_position_setpoint(
                            last_drone_reference[0], last_drone_reference[1],
                            last_drone_reference[2] - SUBTRACTED_HEIGHT, 0)
                        SUBTRACTED_HEIGHT = SUBTRACTED_HEIGHT + DELTA_HEIGHT
                    exit()
