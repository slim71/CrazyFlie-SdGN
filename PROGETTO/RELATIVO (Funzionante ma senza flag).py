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
import math
import argparse
import logging
from own_module import crazyfun as crazy
from datetime import datetime
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

# -----------------------------------------------------------------------------
# ----------------------------------VARIABLES----------------------------------
# -----------------------------------------------------------------------------

# Used in the generation of the position reference for the drone
last_trans = np.array([0, 0, 0])
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

got_frame = 0
last_frame = 0
n_frame = 0

# -----------------------------------------------------------------------------
# ----------------------------------DEBUG--------------------------------------
# -----------------------------------------------------------------------------

# Setting up logs
filename = "./NewLogs/LogFile_CrazyFlie_NOFLAG" \
           + datetime.now().strftime("%Y%m%d_%H%M%S")

logname = filename + ".log"
filename = filename + ".txt"

# Only logs of level ERROR or above will be tracked
# https://docs.python.org/3/library/logging.html#levels
logging.basicConfig(filename=logname,
                    level=logging.DEBUG,
                    filemode='a',
                    format="%(asctime)s [%(levelname)s]: %(message)s")

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

# Connect to the VICON Server running on the same LAN
try:
    vicon.Connect(VICON_IP + ":" + VICON_PORT)
except ViconDataStream.DataStreamException as exc:
    exit("Can't connect to Vicon! Error: ", exc)
    logging.error("Can't connect to Vicon! --> %s", exc)
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
    logging.error("Couldn't setup data types. --> %s", exc)

# Report whether the data types have been enabled
logging.debug('Segments: %s', str(vicon.IsSegmentDataEnabled()))
logging.debug('Markers: %s', str(vicon.IsMarkerDataEnabled()))
logging.debug('Unlabeled Markers: %s',
              str(vicon.IsUnlabeledMarkerDataEnabled()))
logging.debug('Marker Rays: %s', str(vicon.IsMarkerRayDataEnabled()))
logging.debug('Devices: %s', str(vicon.IsDeviceDataEnabled()))
logging.debug('Centroids: %s ', str(vicon.IsCentroidDataEnabled()))

#           Trying to set different stream modes
logging.info("Testing stream modes...")
# "ClientPull": Increases latency, network bandwidth kept at minimum,
# buffers unlikely to be filled up
logging.info("Getting a frame in ClientPull mode...")
try:
    vicon.SetStreamMode(ViconDataStream.Client.StreamMode.EClientPull)
    logging.debug("Fetched! Pulled frame number: %d" if vicon.GetFrame()
                  else "Vicon is not streaming!", vicon.GetFrameNumber())
except ViconDataStream.DataStreamException as exc:
    logging.error("Error using ClientPull mode. --> %s", exc)
# "ClientPreFetch": improved ClientPull, server performances unlikely to be
# affected, latency slightly reduced, buffers unlikely to be filled up
logging.info("Getting a frame in PreFetch mode...")
try:
    vicon.SetStreamMode(ViconDataStream.Client.StreamMode.EClientPullPreFetch)
    logging.debug("Fetched! Pulled frame number: %d" if vicon.GetFrame()
                  else "Vicon is not streaming!", vicon.GetFrameNumber())
except ViconDataStream.DataStreamException as exc:
    logging.error("Error using ClientPreFetch mode. --> %s", exc)
# "ServerPush": the servers pushes frames to the client, best for latency,
# frames dropped only if all buffers are full
logging.info("Getting a frame in ServerPush mode...")
# Continue until this mode is set and working, since it's the one we intend
# to use
while not got_frame:
    try:
        vicon.SetStreamMode(ViconDataStream.Client.StreamMode.EServerPush)
        got_frame = vicon.GetFrame()
        logging.debug("Fetched! Pulled frame number: %d" if got_frame
                      else "Vicon is not streaming!", vicon.GetFrameNumber())
    except ViconDataStream.DataStreamException as exc:
        logging.error("Error using ServerPush mode. --> %s", exc)
logging.info("ServerPush mode set.")
got_frame = 0

# Show the frame rate from both client and server side
logging.info("Getting available framerates...")
try:
    logging.info('Current Vicon frame rate: %s Hz.', vicon.GetFrameRate())
    logging.info('Available frame rates: ')
    for frameRateName, frameRateValue in vicon.GetFrameRates().items():
        logging.info("%s : %d Hz \n", frameRateName, frameRateValue)
except ViconDataStream.DataStreamException as exc:
    logging.error("Framerate error. --> %s", exc)

# Setting reference system
logging.info("Setting axis...")
try:
    vicon.SetAxisMapping(ViconDataStream.Client.AxisMapping.EForward,
                         ViconDataStream.Client.AxisMapping.ELeft,
                         ViconDataStream.Client.AxisMapping.EUp)
except ViconDataStream.DataStreamException as exc:
    logging.error("Error while setting axis. --> %s", exc)

xAxis, yAxis, zAxis = vicon.GetAxisMapping()
logging.info('X Axis: %s', str(xAxis))
logging.info('Y Axis: %s', str(yAxis))
logging.info('Z Axis: %s', str(zAxis))

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
cf = cflib.crazyflie.Crazyflie()

# TODO: move error handling from inside?
first_pos = crazy.getFirstPosition(vicon, drone)

# variable initialization
last_drone_pos = first_pos  # np.array([0.0, 0.0, 0.0])
last_drone_ref = np.array([0.0, 0.0, 0.0])
last_wand_trans = np.array([0.0, 0.0, 0.0])
wand_trans_m = np.array([0.0, 0.0, 0.0])
drone_trans_m = np.array([0.0, 0.0, 0.0])
drone_trans = np.array([0.0, 0.0, 0.0])
wand_trans = np.array([0.0, 0.0, 0.0])
Wand_Translation = np.array([0.0, 0.0, 0.0])

# Test to check Segment and Subject names
subs = vicon.GetSubjectNames()
for each_sub in subs:
    segs = vicon.GetSegmentNames(each_sub)
    logging.info("%s has the following segments: %s", each_sub, segs)
    root = vicon.GetSubjectRootSegmentName(each_sub)
    logging.info("%s has root: %s", str(each_sub), str(root))

logging.debug("Checking functions without flying")
number = 0
while number < 10:
    try:
        got_frame = vicon.GetFrame()
        logging.debug("Fetched! Pulled frame number: %d" if got_frame
                      else "Vicon is not streaming!",
                      vicon.GetFrameNumber())
        if not got_frame:
            continue
    except ViconDataStream.DataStreamException as exc:
        logging.error("Error! --> %s", exc)
        exit("There was an error...")

    # R_x * (R_y * R_z)
    # all in Global Vicon -> RPY in global
    try:
        mat_gl = vicon.GetSegmentGlobalRotationMatrix(drone, drone)
        mat_gl = np.array(mat_gl[0])
        dronePos_l = np.transpose(mat_gl) * drone_trans_m
        drone_trans_m = dronePos_l  # to keep on with the program
        logging.debug("Drone rotation matrix from Vicon: %s %s %s ",
                      str(mat_gl[0]), str(mat_gl[1]), str(mat_gl[2]))
    except ViconDataStream.DataStreamException as exc:
        logging.error("Error! --> %s", exc)
        exit("There was an error...")

    try:
        eul = vicon.GetSegmentGlobalRotationEulerXYZ(drone, drone)
        eul = np.array(eul[0])
        logging.debug("Drone Euler XYZ from Vicon as given: %s",
                      str(eul))
        logging.debug("Drone Euler XYZ from Vicon to deg: (%s, %s, %s)",
                      str(math.degrees(eul[0])),
                      str(math.degrees(eul[0])),
                      str(math.degrees(eul[0])))
        logging.debug("Drone Euler XYZ from Vicon to rad: (%s, %s, %s)",
                      str(math.radians(eul[0])),
                      str(math.radians(eul[0])),
                      str(math.radians(eul[0])))
    except ViconDataStream.DataStreamException as exc:
        logging.error("Error! --> %s", exc)
        exit("There was an error...")

    try:
        drone_trans = vicon.GetSegmentGlobalTranslation(drone, drone)
        logging.debug("Drone position from global origin: %s",
                      str(drone_trans[0]))
    except ViconDataStream.DataStreamException as exc:
        logging.error("Error! --> %s", exc)
        exit("There was an error...")
    number += 1

logging.debug("Offline check done")

# TODO: add check on flowdeck presence? (and catch)
# Class used to start the synchronization with the drone
with SyncCrazyflie(uri, cf) as scf:  # automatic connection
    logging.info("Connected!")

    height_drone = 0

    # AUTO TAKE-OFF to DEFAULT_HEIGHT!
    # Class used for the position control during the take-off phase:
    # take-off automatic when context created using "with"
    with MotionCommander(scf, DEFAULT_HEIGHT) as mc:
        # Functions called during take-off include a waiting period for the
        # drone to reach set height; atm that's commented out!

        logging.info('===========Take-Off!==============')
        mc.forward(0.3)
        print("forward done")
        mc.back(0.3)
        print("back done")
        time.sleep(5)

        # Checking drone height while taking off?
        # Needed if sleep_time is commented out from the take-off
        # (take-off tracks down to line 280 motion_commander.py)
        # IMP: this is necessary to ensure the drone reaches the desired
        # height before computing the flight frames
        # while height_drone < DEFAULT_HEIGHT or not got_frame:
        #
        #     # Request a new data frame from the server and its ordinal
        #     # number
        #     logging.info("\n Getting a frame during take-off...")
        #     try:
        #         got_frame = vicon.GetFrame()
        #         logging.debug("Fetched! Pulled frame number: %d" if got_frame
        #                       else "Vicon is not streaming!",
        #                       vicon.GetFrameNumber())
        #         if not got_frame:
        #             continue
        #     except ViconDataStream.DataStreamException as exc:
        #         logging.error("Error! --> %s", exc)
        #         exit("There was an error...")
        #
        #     # Get the drone position in the Vicon reference system and
        #     # convert to meters
        #     logging.info("Getting drone translation...")
        #     try:
        #         drone_trans = vicon.GetSegmentGlobalTranslation(drone, drone)
        #         logging.debug(" Done.")
        #     except ViconDataStream.DataStreamException as exc:
        #         logging.error("Error! --> %s", exc)
        #         exit("There was an error...")
        #     drone_trans_mm = drone_trans[0]
        #     drone_trans_m = np.array([float(drone_trans_mm[0]) / 1000,
        #                               float(drone_trans_mm[1]) / 1000,
        #                               float(drone_trans_mm[2]) / 1000])
        #
        #     # R_x * (R_y * R_z)
        #     # all in Global Vicon -> RPY in global
        #     try:
        #         mat_gl = vicon.GetSegmentGlobalRotationMatrix(drone, drone)
        #         mat_gl = np.array(mat_gl[0])
        #         # dronePos_l = np.transpose(mat_gl) * drone_trans_m
        #         # drone_trans_m = dronePos_l  # to keep on with the program
        #     except ViconDataStream.DataStreamException as exc:
        #         logging.error("Error! --> %s", exc)
        #         exit("There was an error...")
        #
        #     # Get the Wand position in the Vicon reference system and
        #     # convert to meters
        #     logging.info("Getting Wand translation...")
        #     try:
        #         wand_trans = vicon.GetSegmentGlobalTranslation(Wand, 'Root')
        #         logging.info("Done.")
        #     except ViconDataStream.DataStreamException as exc:
        #         logging.error("Error! --> %s", exc)
        #         exit("There was an error...")
        #     wand_trans_mm = wand_trans[0]
        #     wand_trans_m = np.array([float(wand_trans_mm[0]) / 1000,
        #                              float(wand_trans_mm[1]) / 1000,
        #                              float(wand_trans_mm[2]) / 1000])
        #
        #     height_drone = drone_trans_m[2]
        #
        # last_drone_pos = drone_trans_m
        # last_drone_ref = drone_trans_m
        # last_wand_trans = wand_trans_m  # TODO: first Wand pos?
        #
        # logging.info("Printing available data...")
        # logging.debug("Wand current position (in Vicon system): %s",
        #               str(wand_trans_m))
        # logging.debug("Wand last position (in Vicon system): %s",
        #               str(last_wand_trans))
        # logging.debug("Drone current position (in Vicon system): %s",
        #               str(drone_trans_m))
        # logging.debug("Drone last position (in Vicon system): %s",
        #               str(last_drone_pos))
        # logging.debug("Drone last reference point (in Vicon system): %s",
        #               str(last_drone_ref))
        #
        # time.sleep(5)
        logging.info("==================Core===================")
        while 1:

            # Get the Wand position expressed in the Vicon
            # system and convert it to meters
            logging.info("Getting a frame during flight...")
            try:
                got_frame = 0
                while not got_frame:
                    got_frame = vicon.GetFrame()
                    n_frame = vicon.GetFrameNumber()
                    logging.debug("Fetched! Pulled frame number: %d"
                                  if got_frame
                                  else "Vicon is not streaming!",
                                  n_frame)
                if n_frame - last_frame < 10:
                    continue
                else:
                    last_frame = n_frame

            except ViconDataStream.DataStreamException as exc:
                logging.error("Error! --> %s", exc)
                exit("There was an error...")

            # Matrix_homogeneous, Matrix_Rotation, last_gamma = \
            #     crazy.createMatrixRotation(
            #         vicon,
            #         drone,
            #         first_pos,
            #         last_gamma,
            #         0)  # manual value of the flag to include quaternions in KF
            # vicon_matrix = vicon.GetSegmentGlobalRotationMatrix(drone,
            #                                                     'Crazyflie')
            # Matrix_Rotation = np.asarray(vicon_matrix[0])

            # Absolute position of the Wand in [mm], then converted in [m]
            logging.info("Getting Wand translation...")
            try:
                wand_trans = vicon.GetSegmentGlobalTranslation(Wand, 'Root')
                logging.info("Done.")
            except ViconDataStream.DataStreamException as exc:
                logging.error("Error! --> %s", exc)
                exit("There was an error...")
            wand_trans_mm = wand_trans[0]
            wand_trans_m = np.array([float(wand_trans_mm[0]) / 1000,
                                     float(wand_trans_mm[1]) / 1000,
                                     float(wand_trans_mm[2]) / 1000])

            # Assuming the Wand sends (0, 0, 0) when turned off,
            # this is considered as an error
            if not all(wand_trans_m):  # wand_trans_m == [0, 0, 0]:
                CONSEC_LOSSES += 1
                logging.debug("Fault! Consecutive losses: %d", CONSEC_LOSSES)

                # x, y, z, yaw; wrt the global origin
                # Setpoint, as where the drone will go
                # cf.commander.send_position_setpoint(last_drone_ref[0],
                #                                     last_drone_ref[1],
                #                                     last_drone_ref[2],
                #                                     0)

                # Should apply the homogeneous matrix, but it's null anyway
                Wand_Translation = np.array([0, 0, 0])
                last_wand_trans = wand_trans_m

                if CONSEC_LOSSES == crazy.MAX_LOSS:
                    # LANDING PHASE: We received MAX_LOSS consecutive
                    # null positions of the Wand, so we decide to land
                    logging.error("START LANDING! %d consecutive "
                                  "null positions received.",
                                  crazy.MAX_LOSS)
                    exit("There was an error: drone landed...")

            else:
                logging.info("!!Normal cycle execution!!")

                CONSEC_LOSSES = 0

                # We get the actual position of drone expressed in
                # the Body frame and send it to the Kalman Filter
                logging.info("Getting drone translation...")
                try:
                    drone_trans = vicon.GetSegmentGlobalTranslation(
                        drone, 'Crazyflie')
                    logging.info("Done.")
                except ViconDataStream.DataStreamException as exc:
                    logging.error("Error! --> %s", exc)
                    exit("There was an error...")
                drone_trans_mm = drone_trans[0]
                drone_trans_m = np.array([float(drone_trans_mm[0]) / 1000,
                                          float(drone_trans_mm[1]) / 1000,
                                          float(drone_trans_mm[2]) / 1000])

                logging.info("Getting drone rotation matrix from Vicon...")
                # R_x * (R_y * R_z)
                # all in Global Vicon -> RPY in global
                try:
                    mat_gl = vicon.GetSegmentGlobalRotationMatrix(drone,
                                                                  'Crazyflie')
                    mat_gl = np.array(mat_gl[0])
                    # dronePos_l = np.transpose(mat_gl) * drone_trans_m
                    # drone_trans_m = dronePos_l  # to keep on with the program
                except ViconDataStream.DataStreamException as exc:
                    logging.error("Error! --> %s", exc)
                    exit("There was an error...")

                logging.info("Getting drone Euler angles from Vicon...")
                try:
                    eul = vicon.GetSegmentGlobalRotationEulerXYZ(drone,
                                                                 'Crazyflie')
                    eul = np.array(eul[0])
                except ViconDataStream.DataStreamException as exc:
                    logging.error("Error! --> %s", exc)
                    exit("There was an error...")

                # (The first time this will be 000 in theory)
                # Convert to body reference
                Wand_Translation = np.transpose(mat_gl).dot(wand_trans_m - last_wand_trans)
                # Wand_Translation = np.array(
                #     np.dot(Matrix_Rotation,
                #            np.transpose(Wand_Translation)))

                last_dp = last_drone_pos
                last_dr = last_drone_ref
                last_wp = last_wand_trans

                last_wand_trans = wand_trans_m
                last_drone_ref += Wand_Translation
                last_drone_pos = drone_trans_m

                # Send to KF
                cf.extpos.send_extpos(drone_trans_m[0],
                                      drone_trans_m[1],
                                      drone_trans_m[2])

                # Send the new setpoint to the drone
                cf.commander.send_position_setpoint(last_drone_ref[0],
                                                    last_drone_ref[1],
                                                    last_drone_ref[2],
                                                    eul[2])  # 0)

            logging.info("Printing available data...")
            logging.debug("Wand current position (in Vicon system): %s",
                          str(wand_trans_m))
            logging.debug("Wand last position (in Vicon system): %s",
                          str(last_wp))
            logging.debug("Wand_Translation (in Vicon system): %s",
                          str(Wand_Translation))
            logging.debug("Drone current position (in Vicon system): %s",
                          str(drone_trans_m))
            logging.debug("Drone last position (in Vicon system): %s",
                          str(last_dp))
            logging.debug("Drone current reference point (in Vicon system): %s",
                          str(last_drone_ref))
            logging.debug("Drone last reference point (in Vicon system): %s",
                          str(last_dr))
            # logging.debug("Computed homogeneous matrix: %s",
            # str(Matrix_homogeneous))
            logging.debug("Vicon rotation matrix: %s ",
                          str(mat_gl))
            print("Printing available data...")
            print("Wand current position (in Vicon system): ",
                  str(wand_trans_m), type(wand_trans_m))
            print("Wand last position (in Vicon system): ",
                  str(last_wand_trans), type(last_wand_trans))
            print("Drone current position (in Vicon system): ",
                  str(drone_trans_m), type(drone_trans_m))
            print("Drone last position (in Vicon system): ",
                  str(last_drone_pos), type(last_drone_pos))
            print("Drone last reference point (in Vicon system): ",
                  str(last_drone_ref), type(last_drone_ref))
            # print("Computed homogeneous matrix: ",
            #       str(Matrix_homogeneous), type(Matrix_homogeneous))
            # print("Vicon rotation matrix: ",
            #       str(vicon_matrix[0]), type(vicon_matrix[0]))

    # Automatic land exiting the "with" environment!
