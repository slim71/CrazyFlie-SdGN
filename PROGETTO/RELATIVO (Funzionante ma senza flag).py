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
import crazyfun as crazy

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
last_gamma = 0

WAIT_TIME = 5  # [s]  Used for the management of the iterations
SLEEP_TIME = 0.001  # [s]  Used for the management of the iterations
ITERATIONS = WAIT_TIME / SLEEP_TIME  # Used to manage the iterations
OFFSET = 0.3  # [m]  Security offset

# TODO: move constants and settings to another file (crazyfun?)

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
i = 0

# -----------------------------------------------------------------------------
# ----------------------------------VICON CONNECTION---------------------------
# -----------------------------------------------------------------------------

# Extracted from PyVicon Library:
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--hostname", "--" + VICON_IP + ":" + VICON_PORT)
args = parser.parse_args()

# Create a VICON Client
client = ViconDataStream.Client()
i = 0 # TODO: redeclared?

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


# DRONE CONNECTION
logging.basicConfig(level=logging.ERROR)
cflib.crtp.init_drivers(enable_debug_driver=False)
cf = cflib.crazyflie.Crazyflie()

First_Position = crazy.get_First_Position(client, Drone)
with SyncCrazyflie(uri, cf) as scf:
    with MotionCommander(scf, default_height=0.55) as mc:
        print('TAKE OFF!')
        height_drone = 0
        while height_drone < 0.5:
            a = client.GetFrame()
            b = client.GetFrameNumber()
            D_T_tuple = client.GetSegmentGlobalTranslation(Drone, Drone)
            D_T_millimeters = D_T_tuple[0]
            D_T_meters = np.array([float(D_T_millimeters[0]) / 1000,
                                   float(D_T_millimeters[1]) / 1000,
                                   float(D_T_millimeters[2]) / 1000])
            height_drone = D_T_meters[2]

        last_drone_position = np.array([0, 0, height_drone])
        last_drone_reference = np.array([0, 0, height_drone])
        print(last_drone_reference)
        sdf = 0

        while 1:

            if take_off == 0:  # take off already done
                try:
                    if i == 1:
                        print("Wand position: ", W_T_meters)
                    a = client.GetFrame()
                    b = client.GetFrameNumber()  # It is necessary before
                    # every time we want to call some "GetSegment..()"
                    Matrix_homogeneous, Matrix_Rotation, last_gamma = \
                        crazy.create_Matrix_Rotation(
                            client,
                            Drone,
                            First_Position,
                            last_gamma)
                    W_T_tuple = client.GetSegmentGlobalTranslation(Wand,
                                                                   'Root')
                    # Absolute position of the Drone in [mm]<
                    # We are interested only in the first part of the data
                    # structure
                    W_T_millimeters = W_T_tuple[0]
                    W_T_meters = np.array([float(W_T_millimeters[0]) / 1000,
                                           float(W_T_millimeters[1]) / 1000,
                                           float(W_T_millimeters[
                                                     2]) / 1000])  # Pass
                    # from [mm] to [m]
                    # print("New Wand position: ", W_T_meters)
                    # print("GET FRAME: ",
                    #       W_T_meters[0],
                    #       W_T_meters[1],
                    #       W_T_meters[2])

                    if (W_T_meters[0] == 0 and W_T_meters[1] == 0 and
                            W_T_meters[2] == 0):
                        CONSECUTIVE_LOSS = CONSECUTIVE_LOSS + 1
                        cf.commander.send_position_setpoint(
                            last_drone_reference[0], last_drone_reference[1],
                            last_drone_reference[2], 0)
                        if CONSECUTIVE_LOSS == crazy.MAX_LOSS:
                            # LANDING PHASE: We received MAX_LOSS consecutive
                            # "null position" of the wand so we decide to
                            # start landing.
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
                        # Body frame of the Drone before sending it
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
                            Wand_Translation = np.dot(Matrix_Rotation,
                                                     np.transpose(
                                                         Wand_Translation))
                            print("Body translation: ", Wand_Translation)
                        # print("Drone reference: ", last_drone_reference)
                        last_drone_reference[0] = last_drone_reference[0] + \
                                                  Wand_Translation[0]
                        last_drone_reference[1] = last_drone_reference[1] + \
                                                  Wand_Translation[1]
                        last_drone_reference[2] = last_drone_reference[2] + \
                                                  Wand_Translation[2]
                        print("New Drone reference: ",
                              last_drone_reference, "\n\n")
                        i = 1
                        # We send the actual position of Drone expressed in
                        # the Body frame to the Kalman Filter
                        D_T_tuple = client.GetSegmentGlobalTranslation(Drone,
                                                                       Drone)
                        D_T_millimeters = D_T_tuple[0]
                        D_T_meters = np.array(
                            [float(D_T_millimeters[0]) / 1000,
                             float(D_T_millimeters[1]) / 1000,
                             float(D_T_millimeters[2]) / 1000])
                        if (D_T_meters[0] == 0 and D_T_meters[1] == 0 and
                                D_T_meters[2] == 0):
                            D_T_meters = last_drone_position
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
                            last_drone_position = D_T_meters

                        cf.extpos.send_extpos(D_T_meters[0], D_T_meters[1],
                                              D_T_meters[2])
                        print("Location of the Drone: ", D_T_meters[0],
                              D_T_meters[1], D_T_meters[2])
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
                    print('START LANDING: This error form Vicon occurred: \n',
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
