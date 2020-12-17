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
from datetime import datetime

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
# ----------------------------------DEBUG--------------------------------------
# -----------------------------------------------------------------------------

# Setting up logs
filename = "LogFile_CrazyFlie_NOFLAG" \
           + datetime.now().strftime("%Y%m%d_%H%M%S")

logname = filename + ".log"
filename = filename + ".txt"

# Only logs of level ERROR or above will be tracked
# https://docs.python.org/3/library/logging.html#levels
logging.basicConfig(filename=logname,
                    level=logging.DEBUG,
                    filemode='a',
                    format="%(asctime)s [%(levelname)s]: %(messages)s")

# -----------------------------------------------------------------------------
# ----------------------------------VICON CONNECTION---------------------------
# -----------------------------------------------------------------------------

# Extracted from PyVicon Library
logging.info("Connecting to the Vicon...")

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--hostname", "--" + VICON_IP + ":" + VICON_PORT)
args = parser.parse_args()

# Create a VICON Client
# The Client is implicitly destroyed as it goes out of scope.
vicon = ViconDataStream.Client()
# TODO: i=0
# Connect to the VICON Server running on the same LAN
try:
    vicon.Connect(VICON_IP + ":" + VICON_PORT)
except ViconDataStream.DataStreamException as exc:
    print("Can't connect to Vicon! Error: ")
    logging.error("Can't connect to Vicon! \n %s", exc)

logging.info("Connected to Vicon!")

# Setting a buffer size to work with
logging.info("Setting up the buffer...")
vicon.SetBufferSize(frame_num)
logging.info("Buffer of %d frames created.", frame_num)

# Enable all the data types (action needed to be able to use these)
logging.info("Enabling data types...")

try:
    vicon.EnableSegmentData()
    vicon.EnableMarkerData()
    vicon.EnableUnlabeledMarkerData()
    vicon.EnableMarkerRayData()
    vicon.EnableDeviceData()
    vicon.EnableCentroidData()
except ViconDataStream.DataStreamException as exc:
    logging.error("Couldn't setup data types. \n ", exc)

# Report whether the data types have been enabled
logging.debug('Segments: ', vicon.IsSegmentDataEnabled())
logging.debug('Markers: ', vicon.IsMarkerDataEnabled())
logging.debug('Unlabeled Markers: ', vicon.IsUnlabeledMarkerDataEnabled())
logging.debug('Marker Rays: ', vicon.IsMarkerRayDataEnabled())
logging.debug('Devices: ', vicon.IsDeviceDataEnabled())
logging.debug('Centroids: ', vicon.IsCentroidDataEnabled())

#           Trying to set different stream modes
logging.info("Testing stream modes...")

# "ClientPull": Increases latency, network bandwidth kept at minimum,
# buffers unlikely to be filled up
logging.info("Getting a frame in ClientPull mode... \n")
try:
    vicon.SetStreamMode(ViconDataStream.Client.StreamMode.EClientPull)
    logging.debug("Fetched! Pulled frame number: %d" if vicon.GetFrame()
                  else "Vicon is not streaming!", vicon.GetFrameNumber())
except ViconDataStream.DataStreamException as exc:
    logging.error("Error using ClientPull mode. \n %s", exc)
# "ClientPreFetch": improved ClientPull, server performances unlikely to be
# affected, latency slightly reduced, buffers unlikely to be filled up
logging.info("Getting a frame in PreFetch mode... \n")
try:
    vicon.SetStreamMode(ViconDataStream.Client.StreamMode.EClientPullPreFetch)
    logging.debug("Fetched! Pulled frame number: %d" if vicon.GetFrame()
                  else "Vicon is not streaming!", vicon.GetFrameNumber())
except ViconDataStream.DataStreamException as exc:
    logging.error("Error using ClientPreFetch mode. \n %s", exc)
# "ServerPush": the servers pushes frames to the client, best for latency,
# frames dropped only if all buffers are full
logging.info("Getting a frame in ServerPush mode... \n")
try:
    vicon.SetStreamMode(ViconDataStream.Client.StreamMode.EServerPush)
    logging.debug("Fetched! Pulled frame number: %d" if vicon.GetFrame()
                  else "Vicon is not streaming!", vicon.GetFrameNumber())
except ViconDataStream.DataStreamException as exc:
    logging.error("Error using ServerPush mode. \n %s", exc)

logging.info("ServerPush mode set.")

# Show the frame rate from both client and server side
try:
    logging.info('Current frame rate: %s Hz.', vicon.GetFrameRate())
    logging.info('Frame rates available: ')
    for frameRateName, frameRateValue in vicon.GetFrameRates().items():
        logging.info("%s : %d Hz \n", frameRateName, frameRateValue)
except ViconDataStream.DataStreamException as exc:
    logging.error("Framerate error. \n %s", exc)

# Setting reference system
try:
    vicon.SetAxisMapping(ViconDataStream.Client.AxisMapping.EForward,
                         ViconDataStream.Client.AxisMapping.ELeft,
                         ViconDataStream.Client.AxisMapping.EUp)
except ViconDataStream.DataStreamException as exc:
    logging.error("Error while setting axis. \n %s", exc)

xAxis, yAxis, zAxis = vicon.GetAxisMapping()
logging.info('X Axis: ', xAxis)
logging.info('Y Axis: ', yAxis)
logging.info('Z Axis: ', zAxis)

# -----------------------------------------------------------------------------
# ----------------------------------MAIN---------------------------------------
# -----------------------------------------------------------------------------

# Initialize all the drivers
logging.info("Initializing drivers...")
cflib.crtp.init_drivers(enable_debug_driver=False)
logging.info("Drivers initialized.")

# DRONE CONNECTION
logging.info('Connecting to the Crazyflie...')

# Creating an instance of the Crazyflie object and getting the initial position
cf = cflib.crazyflie.Crazyflie()  # TODO: debug included
# TODO: commander included, +-mode?

# TODO: move error handling from inside?
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
        while height_drone < 0.5 or not got_frame:

            # Request a new data frame from the server and its ordinal
            # number
            logging.info("Getting a frame during initial take-off... \n")
            try:
                got_frame = vicon.GetFrame()
                # frame_num = vicon.GetFrameNumber()
                logging.debug("Fetched! Pulled frame number: %d" if
                              got_frame
                              else "Vicon is not streaming!",
                              vicon.GetFrameNumber())
                if not got_frame:
                    continue
            except ViconDataStream.DataStreamException as exc:
                logging.error("Error! \n %s", exc)

            # Get the drone position in the Vicon reference system and
            # convert to meters
            logging.info("Getting drone translation...")
            try:
                drone_trans = vicon.GetSegmentGlobalTranslation(drone, drone)
                logging.debug(" Done.")
            except ViconDataStream.DataStreamException as exc:
                logging.error("Error! \n %s", exc)
            drone_trans_mm = drone_trans[0]
            drone_trans_m = np.array([float(drone_trans_mm[0]) / 1000,
                                      float(drone_trans_mm[1]) / 1000,
                                      float(drone_trans_mm[2]) / 1000])
            height_drone = drone_trans_m[2]

        last_drone_pos = np.array([0, 0, height_drone])
        last_drone_ref = np.array([0, 0, height_drone])
        # print(last_drone_ref)
        sdf = 0  # TODO: what's this?

        while 1:

            # take_off currently set to 0
            if take_off == 0:  # take off already done
                if i == 1:
                    # TODO: why only a print?
                    logging.debug("i=1 -> Wand position: %s", wand_trans_m)

                # Get the Wand position expressed in the Vicon
                # system and we convert it to meters
                logging.info("Getting a frame during flight...")
                try:
                    got_frame = 0
                    while not got_frame:
                        got_frame = vicon.GetFrame()
                        # frame_num = vicon.GetFrameNumber()
                        logging.debug("Fetched! Pulled frame number: %d" if
                                      got_frame
                                      else "Vicon is not streaming!",
                                      vicon.GetFrameNumber())
                except ViconDataStream.DataStreamException as exc:
                    logging.error("Error! \n %s", exc)

                # TODO: add custom log
                Matrix_homogeneous, Matrix_Rotation, last_gamma = \
                    crazy.createMatrixRotation(
                        vicon,
                        drone,
                        first_pos,
                        last_gamma,
                        1)  # manual value for the log flag

                logging.info("Getting wand translation...")
                try:
                    wand_trans = vicon.GetSegmentGlobalTranslation(Wand,
                                                                   'Root')
                    logging.info("Done.")
                except ViconDataStream.DataStreamException as exc:
                    logging.error("Error! \n %s", exc)

                # Absolute position of the drone in [mm]
                # We are interested only in the first part of the data
                # structure
                wand_trans_mm = wand_trans[0]
                wand_trans_m = np.array([float(wand_trans_mm[0]) / 1000,
                                         float(wand_trans_mm[1]) / 1000,
                                         float(wand_trans_mm[2]) / 1000])

                if (wand_trans_m[0] == 0
                        and wand_trans_m[1] == 0
                        and wand_trans_m[2] == 0):

                    CONSEC_LOSSES += 1
                    logging.debug("Fault! Consecutive losses: %d",
                                  CONSEC_LOSSES)

                    # x, y, z, yaw
                    cf.commander.send_position_setpoint(last_drone_ref[0],
                                                        last_drone_ref[1],
                                                        last_drone_ref[2],
                                                        0)

                    if LOSSES == crazy.MAX_LOSS:
                        # LANDING PHASE: We received MAX_LOSS consecutive
                        # "null position" of the wand, so We decide to
                        # start landing.
                        # TODO: land() with this
                        logging.error("START LANDING! %d consecutive "
                                      "null positions received.",
                                      crazy.MAX_LOSS)

                        while last_drone_ref[2] \
                                - SUBTRACTED_HEIGHT > 0:
                            cf.commander.send_position_setpoint(
                                last_drone_ref[0],
                                last_drone_ref[1],
                                last_drone_ref[2]
                                - SUBTRACTED_HEIGHT,
                                0)
                            SUBTRACTED_HEIGHT = \
                                SUBTRACTED_HEIGHT + DELTA_HEIGHT
                        exit()
                else:
                    # We pass from a Wand position expressed in the
                    # Vicon frame to a Wand position expressed in the
                    # Body frame of the drone, before sending it
                    if i == 0 or LOSSES:

                        Wand_Translation = np.array([0, 0, 0])
                        W_T_prec = np.array(
                            [wand_trans_m[0], wand_trans_m[1],
                             wand_trans_m[2]])
                        LOSSES = 0

                    else:
                        LOSSES = 0
                        Wand_Translation[0] = wand_trans_m[0] - W_T_prec[0]
                        Wand_Translation[1] = wand_trans_m[1] - W_T_prec[1]
                        Wand_Translation[2] = wand_trans_m[2] - W_T_prec[2]
                        W_T_prec = np.array([wand_trans_m[0],
                                             wand_trans_m[1],
                                             wand_trans_m[2]])

                    if i == 1:
                        Wand_Translation = np.dot(
                            Matrix_Rotation,
                            np.transpose(Wand_Translation))

                    last_drone_ref[0] = last_drone_ref[0] + \
                                        Wand_Translation[0]
                    last_drone_ref[1] = last_drone_ref[1] + \
                                        Wand_Translation[1]
                    last_drone_ref[2] = last_drone_ref[2] + \
                                        Wand_Translation[2]

                    i = 1

                    # We get the actual position of drone expressed in
                    # the Body frame and send it to the Kalman Filter
                    logging.info("Getting drone translation...")
                    try:
                        drone_trans = vicon.GetSegmentGlobalTranslation(
                            drone, drone)
                        logging.info("Done.")
                    except ViconDataStream.DataStreamException as exc:
                        logging.error("Error! \n %s", exc)

                    drone_trans_mm = drone_trans[0]
                    drone_trans_m = np.array(
                        [float(drone_trans_mm[0]) / 1000,
                         float(drone_trans_mm[1]) / 1000,
                         float(drone_trans_mm[2]) / 1000])

                    if (drone_trans_m[0] == 0
                            and drone_trans_m[1] == 0
                            and drone_trans_m[2] == 0):
                        drone_trans_m = last_drone_pos
                    else:
                        D_T_homogenous = np.array(
                            [drone_trans_m[0], drone_trans_m[1],
                             drone_trans_m[2],
                             1])
                        D_T_homogenous = np.dot(
                            Matrix_homogeneous,
                            np.transpose(D_T_homogenous))
                        drone_trans_m = np.array(
                            [D_T_homogenous[0], D_T_homogenous[1],
                             D_T_homogenous[2]])
                        last_drone_pos = drone_trans_m

                    # Send to KF
                    cf.extpos.send_extpos(drone_trans_m[0],
                                          drone_trans_m[1],
                                          drone_trans_m[2])

                    # We send the new setpoint
                    cf.commander.send_position_setpoint(
                        last_drone_ref[0], last_drone_ref[1],
                        last_drone_ref[2], 0)
                    sdf = sdf + 0.05

                # except ViconDataStream.DataStreamException as e:
                #     print('START LANDING: This error from Vicon occurred: \n',
                #           e)
                #     # LANDING PHASE: We received MAX_LOSS consecutive
                #     # "null position" of the wand so we decide to start
                #     # landing.
                #     while last_drone_ref[2] - SUBTRACTED_HEIGHT > 0:
                #         cf.commander.send_position_setpoint(
                #             last_drone_ref[0], last_drone_ref[1],
                #             last_drone_ref[2] - SUBTRACTED_HEIGHT, 0)
                #         SUBTRACTED_HEIGHT = SUBTRACTED_HEIGHT + DELTA_HEIGHT
                #     exit()
