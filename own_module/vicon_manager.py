import argparse
import logging
import numpy as np
import time
import math
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


def quaternion_product(q0, q1):  # both Vicon and Crazy-Kalman use the form (x, y, z, w), NOT the common (w, x, y, z)
    x0, y0, z0, w0 = q0
    x1, y1, z1, w1 = q1

    # w0, x0, y0, z0 = q0
    # w1, x1, y1, z1 = q1

    q0q1_w = w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1
    q0q1_x = w0 * x1 + x0 * w1 + y0 * z1 - z0 * y1
    q0q1_y = w0 * y1 - x0 * z1 + y0 * w1 + z0 * x1
    q0q1_z = w0 * z1 + x0 * y1 - y0 * x1 + z0 * w1

    return q0q1_x, q0q1_y, q0q1_z, q0q1_w
    # return q0q1_w, q0q1_x, q0q1_y, q0q1_z


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

        logging.info("Getting available framerates...")  # Show rates of client and server
        for frameRateName, frameRateValue in self._client.GetFrameRates().items():
            logging.info("%s : %d Hz \n", frameRateName, frameRateValue)

        self.set_reference_system()

    def yaw2quat(self, yaw):  # input yaw in degree
        yaw = yaw*math.pi/180
        qx, qy = 0, 0
        qz = math.sin(yaw/2)
        qw = math.cos(yaw/2)
        return qx, qy, qz, qw

    def quat2yaw(self, q):
        yaw = 2 * math.acos(q[3])
        return yaw * 180 / math.pi  # return in degree

    def change2G_coord(self, V2B_transl, V2B_rot, V2G_dist, V2G_rot):
        # transpose the rotation (conjugate the quaternion) to have a coordinate transformation from V to G
        V2G_coord = (-V2G_rot[0], -V2G_rot[1], -V2G_rot[2], V2G_rot[3])
        V2G_coord_conj = V2G_rot

        V2B_transl = np.array((V2B_transl[0], V2B_transl[1], V2B_transl[2], 0))  # get a quaternion from position vector
        V2G_dist = np.array((V2G_dist[0], V2G_dist[1], V2G_dist[2], 0))
        V2B_transl_G = quaternion_product(V2G_coord,
                                          quaternion_product(V2B_transl - V2G_dist, V2G_coord_conj))
        Btransl_G = np.array([V2B_transl_G[0], V2B_transl_G[1], V2B_transl_G[2]])
        Btransl_G = (float(Btransl_G[0])/1000, float(Btransl_G[1])/1000, float(Btransl_G[2])/1000)

        Brot_G = quaternion_product(V2G_coord, V2B_rot)  # Body rotation in Global coordinates
        # Doubles cast to float since packets must be limited to 32bytes: now extpose will send (7 variables)x(4 bytes)
        Brot_G = (float(Brot_G[0]), -float(Brot_G[1]), float(Brot_G[2]), float(Brot_G[3]))
        # qy is negated since (ONLY) pitch is left-handed
        return Btransl_G, Brot_G

    def get_global_frame(self):
        self._client.GetFrame()
        # initial translation and rotation of Drone Global frame (i.e. body at t0) in Vicon coordinates
        V2G_dist = np.array(self._client.GetSegmentGlobalTranslation(Drone, "Crazyflie2")[0])
        V2G_rot = np.array(self._client.GetSegmentGlobalRotationQuaternion(Drone, "Crazyflie2")[0])
        return V2G_dist, V2G_rot

    def get_drone_pose(self, V2G_dist, V2G_rot):  # return pose wrt its global frame (ready for Kalman)
        self._client.GetFrame()
        V2B_transl = np.array(self._client.GetSegmentGlobalTranslation(Drone, "Crazyflie2")[0])
        V2B_rot = np.array(self._client.GetSegmentGlobalRotationQuaternion(Drone, "Crazyflie2")[0])
        # print("V2B_transl: ", str(V2B_transl), " V2B_rot: ", str(V2B_rot))
        return self.change2G_coord(V2B_transl, V2B_rot, V2G_dist, V2G_rot)

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
