import os
import logging
from datetime import datetime
from cflib import crazyflie
from own_module import script_variables as sc_v
from cflib import crtp
from vicon_dssdk import ViconDataStream

# -----------------------------------DEBUG------------------------------------
file_name = os.path.normpath(__file__).split(os.sep)[-1][:-3]

filename = "../experiment_logs/" + file_name + \
           datetime.now().strftime("__%Y%m%d_%H%M%S")
logname = filename + ".log"

# Only logs of level ERROR or above will be tracked
# ref: https://docs.python.org/3/library/logging.html#levels
logging.basicConfig(filename=logname,
                    level=logging.DEBUG,
                    filemode='a',
                    format="%(asctime)s [%(levelname)s]: %(message)s")

# -------------------------------CONNECTION-----------------------------------
logging.info("Connecting to the Vicon...")

# Create a VICON Client
# (The client is implicitly destroyed as it goes out of scope)
vicon = ViconDataStream.Client()

# Connect to the VICON Server running on the same LAN
try:
    vicon.Connect(sc_v.VICON_IP + ":" + sc_v.VICON_PORT)
except ViconDataStream.DataStreamException as exc:
    logging.error("Can't connect to Vicon! --> %s", str(exc))
    exit("Can't connect to Vicon! --> " + str(exc))

logging.info("Connected to Vicon!")

# -----------------------------VICON SETTINGS---------------------------------
# Setting a buffer size to work with
# logging.info("Setting up the buffer...")
# vicon.SetBufferSize(sc_v.buffer_size)
# logging.info("Buffer of %d frames created.", sc_v.buffer_size)

# Enable all the data types (action needed to use them)
logging.info("Enabling data types...")
try:
    vicon.EnableSegmentData()
    vicon.EnableMarkerData()
    vicon.EnableUnlabeledMarkerData()
    vicon.EnableMarkerRayData()
    vicon.EnableDeviceData()
    vicon.EnableCentroidData()
except ViconDataStream.DataStreamException as exc:
    logging.error("Couldn't setup data types --> %s", str(exc))
    # Report data types toggle status
    logging.debug('Segments: %s', str(vicon.IsSegmentDataEnabled()))
    logging.debug('Markers: %s', str(vicon.IsMarkerDataEnabled()))
    logging.debug('Unlabeled Markers: %s',
                  str(vicon.IsUnlabeledMarkerDataEnabled()))
    logging.debug('Marker Rays: %s', str(vicon.IsMarkerRayDataEnabled()))
    logging.debug('Devices: %s', str(vicon.IsDeviceDataEnabled()))
    logging.debug('Centroids: %s ', str(vicon.IsCentroidDataEnabled()))
    exit("Couldn't setup data types --> " + str(exc))

logging.info("Data types enabled")

# Attempts to set different stream modes
logging.info("Testing stream modes...")
# 1. "ClientPull": increases latency; network bandwidth kept at minimum;
#                  buffers unlikely to be filled up
logging.info("Getting a frame in ClientPull mode...")
try:
    vicon.SetStreamMode(ViconDataStream.Client.StreamMode.EClientPull)
    logging.debug("Fetched! Pulled frame number: %d" if vicon.GetFrame()
                  else "Vicon is not streaming in ClientPull mode!",
                  vicon.GetFrameNumber())
except ViconDataStream.DataStreamException as exc:
    logging.error("Error using ClientPull mode --> %s", str(exc))
logging.info("ClientPull mode available.")

# 2. "ClientPreFetch": improved ClientPull; server performances unlikely to be
#                      affected; latency slightly reduced; buffers unlikely to 
#                      be filled up
logging.info("Getting a frame in ClientPreFetch mode...")
try:
    vicon.SetStreamMode(ViconDataStream.Client.StreamMode.EClientPullPreFetch)
    logging.debug("Fetched! Pulled frame number: %s" if vicon.GetFrame()
                  else "Vicon is not streaming in ClientPreFetch mode!",
                  str(vicon.GetFrameNumber()))
except ViconDataStream.DataStreamException as exc:
    logging.error("Error using ClientPreFetch mode --> %s", str(exc))
logging.info("ClientPreFetch mode available.")

# 3. "ServerPush": the servers pushes frames to the client; best for latency,
#                  frames dropped only if all buffers are full
#
# Continue until ServerPush mode is set and working, since it's the one we
# intend to use
logging.info("Getting a frame in ServerPush mode...")
while not sc_v.got_frame:
    try:
        vicon.SetStreamMode(ViconDataStream.Client.StreamMode.EServerPush)
        sc_v.got_frame = vicon.GetFrame()
        logging.debug("Fetched! Pulled frame number: %d" if sc_v.got_frame
                      else "Vicon is not streaming in ServerPush mode!",
                      vicon.GetFrameNumber())
    except ViconDataStream.DataStreamException as exc:
        logging.error("Error using ServerPush mode --> %s", str(exc))
logging.info("ServerPush mode set.")
sc_v.got_frame = 0

# Show the framerate from both client and server side
logging.info("Getting available framerates...")
try:
    logging.info('Current Vicon framerate: %s Hz.', vicon.GetFrameRate())
    logging.info('Available framerates:')
    for frameRateName, frameRateValue in vicon.GetFrameRates().items():
        logging.info("%s : %d Hz \n", str(frameRateName), frameRateValue)
except ViconDataStream.DataStreamException as exc:
    logging.error("Framerate error. --> %s", str(exc))

# Setting reference system (Vicon standard: X forward, Y left, Z up)
logging.info("Setting axis...")
try:
    vicon.SetAxisMapping(ViconDataStream.Client.AxisMapping.EForward,
                         ViconDataStream.Client.AxisMapping.ELeft,
                         ViconDataStream.Client.AxisMapping.EUp)
except ViconDataStream.DataStreamException as exc:
    logging.error("Error while setting axis. --> %s", str(exc))
# Check and report Axis
xAxis, yAxis, zAxis = vicon.GetAxisMapping()
logging.info('X axis: %s', str(xAxis))
logging.info('Y axis: %s', str(yAxis))
logging.info('Z axis: %s', str(zAxis))

# Initialize all the drivers
logging.info("Initializing drivers...")
crtp.init_drivers(enable_debug_driver=False)
logging.info("Drivers initialized.")

# ----------------------------DRONE CONNECTION--------------------------------
logging.info('Connecting to the Crazyflie...')

# Creating an instance of the Crazyflie object
cf = crazyflie.Crazyflie()

# -----------------------------VICON CHECKS-----------------------------------

# Test to check Segment and Subject names in Vicon
subs = vicon.GetSubjectNames()
for each_sub in subs:
    segs = vicon.GetSegmentNames(each_sub)
    logging.info("%s has the following segments: %s", str(each_sub), str(segs))
    root = vicon.GetSubjectRootSegmentName(each_sub)
    logging.info("%s has root: %s", str(each_sub), str(root))

logging.debug("===============SETUP DONE=================")
