import argparse
import logging

import numpy as np
from vicon_dssdk import ViconDataStream

VICON_IP = "192.168.0.2"  # Set the IP of the Vicon server to connect to
VICON_PORT = "801"  # Set the port of the Vicon server to connect to
Drone = "Crazyflie2"  # Drone "Vicon name"
Wand = "Active Wand v2 (Origin Tracking)"  # Wand "Vicon-name"

FRAME_NUM = 10

# Extracted from PyVicon Library: (?)
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--hostname", "--" + VICON_IP + ":" + VICON_PORT)
args = parser.parse_args()


class ViconManager:

    # Create Client and connect to Server
    def __init__(self):
        self._client = ViconDataStream.Client()
        try:
            self._client.Connect(VICON_IP + ":" + VICON_PORT)
        except ViconDataStream.DataStreamException as exc:
            exit("Can't connect to Vicon! Error: ", exc)

        self._client.SetBufferSize(FRAME_NUM)  # 1000
        logging.info("Buffer of %d frames created.", FRAME_NUM)

        self.enable_data()

        got_frame = 0
        while not got_frame:  # Continue until the stream mode is set and working
            try:
                self._client.SetStreamMode(
                    ViconDataStream.Client.StreamMode.EServerPush)
                got_frame = self._client.GetFrame()
                logging.debug("Fetched! Pulled frame number: %d" if got_frame
                              else "Vicon is not streaming!",
                              self._client.GetFrameNumber())
            except ViconDataStream.DataStreamException as exc:
                exit("Error using ServerPush mode. --> %s", exc)

        logging.info(
            "Getting available framerates...")  # Show rates of client and server
        for frameRateName, frameRateValue in self._client.GetFrameRates().items():
            logging.info("%s : %d Hz \n", frameRateName, frameRateValue)

        self.set_reference_system()

    def get_drone_pos(self):  # TODO check vicon vs crazy frame
        self._client.GetFrame()
        drone_trans = self._client.GetSegmentGlobalTranslation(Drone, "Crazyflie2")
        drone_trans_mm = drone_trans[0]
        drone_trans_m = np.array([float(drone_trans_mm[0]) / 1000,
                                  float(drone_trans_mm[1]) / 1000,
                                  float(drone_trans_mm[2]) / 1000])
        # con X-drone lungo Y-vicon
        # drone_trans_m = np.array([float(drone_trans_mm[1]) / 1000,
        #                           -float(drone_trans_mm[0]) / 1000,
        #                           float(drone_trans_mm[2]) / 1000])
        return drone_trans_m

    def enable_data(self):  # Enable all data types
        try:
            self._client.EnableSegmentData()
            self._client.EnableMarkerData()
            self._client.EnableUnlabeledMarkerData()
            self._client.EnableMarkerRayData()
            self._client.EnableDeviceData()
            self._client.EnableCentroidData()
        except ViconDataStream.DataStreamException as exc:
            logging.error("Couldn't setup data types. --> %s", exc)

    def set_reference_system(self):
        try:
            self._client.SetAxisMapping(
                ViconDataStream.Client.AxisMapping.EForward,
                ViconDataStream.Client.AxisMapping.ELeft,
                ViconDataStream.Client.AxisMapping.EUp)
        except ViconDataStream.DataStreamException as exc:
            logging.error("Error while setting axis. --> %s", exc)
        xAxis, yAxis, zAxis = self._client.GetAxisMapping()
        logging.info('X Axis: %s', str(xAxis))
        logging.info('Y Axis: %s', str(yAxis))
        logging.info('Z Axis: %s', str(zAxis))
