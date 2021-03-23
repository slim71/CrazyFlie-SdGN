import os
import __main__
import logging
from datetime import datetime
import numpy as np
from cflib import crazyflie
import script_variables as sc_v
from cflib import crtp
from vicon_dssdk import ViconDataStream

# -----------------------------------DEBUG------------------------------------
file_name = os.path.normpath(__main__.__file__).split(os.sep)[-1][:-3]

filename = "./exp_logs/" + file_name + \
           datetime.now().strftime("__%Y%m%d_%H%M%S")
logname = filename + ".log"

# Only logs of level ERROR or above will be tracked
# https://docs.python.org/3/library/logging.html#levels
logging.basicConfig(filename=logname,
                    level=logging.DEBUG,
                    filemode='a',
                    format="%(asctime)s [%(levelname)s]: %(message)s")

# -------------------------------CONNECTION-----------------------------------

# Extracted from PyVicon Library
logging.info("Connecting to the Vicon...")

# Create a VICON Client
# (The client is implicitly destroyed as it goes out of scope)
vicon = ViconDataStream.Client()

# Connect to the VICON Server running on the same LAN
try:
    vicon.Connect(sc_v.VICON_IP + ":" + sc_v.VICON_PORT)
except ViconDataStream.DataStreamException as exc:
    logging.error("Can't connect to Vicon! --> %s", exc)
    exit("Can't connect to Vicon! --> " + str(exc))
logging.info("Connected to Vicon!")

# Setting a buffer size to work with
# logging.info("Setting up the buffer...")
# vicon.SetBufferSize(sc_v.buffer_size)
# logging.info("Buffer of %d frames created.", sc_v.buffer_size)

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
while not sc_v.got_frame:
    try:
        vicon.SetStreamMode(ViconDataStream.Client.StreamMode.EServerPush)
        sc_v.got_frame = vicon.GetFrame()
        logging.debug("Fetched! Pulled frame number: %d" if sc_v.got_frame
                      else "Vicon is not streaming!", vicon.GetFrameNumber())
    except ViconDataStream.DataStreamException as exc:
        logging.error("Error using ServerPush mode. --> %s", exc)
logging.info("ServerPush mode set.")

# Show the frame rate from both client and server side
logging.info("Getting available framerates...")
try:
    logging.info('Current Vicon frame rate: %s Hz.', vicon.GetFrameRate())
    logging.info('Available frame rates: ')
    for frameRateName, frameRateValue in vicon.GetFrameRates().items():
        logging.info("%s : %d Hz \n", frameRateName, frameRateValue)
except ViconDataStream.DataStreamException as exc:
    logging.error("Framerate error. --> %s", exc)

# Setting reference system (Vicon standard: X forward, Y left, Z up)
logging.info("Setting axis...")
try:
    vicon.SetAxisMapping(ViconDataStream.Client.AxisMapping.EForward,
                         ViconDataStream.Client.AxisMapping.ELeft,
                         ViconDataStream.Client.AxisMapping.EUp)
except ViconDataStream.DataStreamException as exc:
    logging.error("Error while setting axis. --> %s", exc)
# Check and report Axis
xAxis, yAxis, zAxis = vicon.GetAxisMapping()
logging.info('X Axis: %s', str(xAxis))
logging.info('Y Axis: %s', str(yAxis))
logging.info('Z Axis: %s', str(zAxis))

# Initialize all the drivers
logging.info("Initializing drivers...")
crtp.init_drivers(enable_debug_driver=False)
logging.info("Drivers initialized.")

# DRONE CONNECTION
logging.info('Connecting to the Crazyflie...')

# Creating an instance of the Crazyflie object
cf = crazyflie.Crazyflie()

# Test to check Segment and Subject names
subs = vicon.GetSubjectNames()
for each_sub in subs:
    segs = vicon.GetSegmentNames(each_sub)
    logging.info("%s has the following segments: %s", each_sub, segs)
    root = vicon.GetSubjectRootSegmentName(each_sub)
    logging.info("%s has root: %s", str(each_sub), str(root))

logging.debug("===Checking functions without flying===")
attempts = 0
got_frame = 0  # reset flag
# while attempts < 10:
#     try:
#         got_frame = vicon.GetFrame()
#         logging.debug("Fetched! Pulled frame number: %d" if got_frame
#                       else "Vicon is not streaming!", vicon.GetFrameNumber())
#         if not got_frame:
#             continue
#     except ViconDataStream.DataStreamException as exc:
#         exit("Error while getting frame in check mode: " + str(exc))
#         logging.error("Error while getting frame in check mode! --> %s", exc)
#
#     # all in Global Vicon -> RPY in global
#     try:
#         mat_gl = vicon.GetSegmentGlobalRotationMatrix(sc_v.drone, sc_v.drone)
#         # logging.debug("Drone rotation matrix from Vicon: %s %s %s ",
#         #               str(mat_gl[0]), str(mat_gl[1]), str(mat_gl[2]))
#         logging.debug("Drone rotation matrix from Vicon: %s",
#                       str(mat_gl))
#     except ViconDataStream.DataStreamException as exc:
#         exit("Error while managing the rotation: " + str(exc))
#         logging.error("Error while managing the rotation! --> %s", exc)
#
#     try:
#         eul = vicon.GetSegmentGlobalRotationEulerXYZ(sc_v.drone, sc_v.drone)
#         eul = np.array(eul[0])
#         logging.debug("Drone Euler XYZ from Vicon as given: %s", str(eul))
#     except ViconDataStream.DataStreamException as exc:
#         exit("Error while managing Euler angles: " + str(exc))
#         logging.error("Error while managing Euler angles! --> %s", exc)
#
#     try:
#         drone_trans = vicon.GetSegmentGlobalTranslation(sc_v.drone, sc_v.drone)
#         logging.debug("Drone position from global origin: %s",
#                       str(drone_trans[0]))
#     except ViconDataStream.DataStreamException as exc:
#         exit("Error while getting drone translation: " + str(exc))
#         logging.error("Error while getting drone translation! --> %s", exc)
#     attempts += 1

logging.debug("===============Offline check done=================")
