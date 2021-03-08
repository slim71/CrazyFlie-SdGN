from __future__ import print_function
from vicon_dssdk import ViconDataStream
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig
from cflib.positioning.motion_commander import MotionCommander
import numpy as np
import logging
import script_variables as sc_v
import script_setup as sc_s
from own_module import crazyfun as crazy

# Class used to start the synchronization with the drone
with SyncCrazyflie(sc_v.uri, sc_s.cf) as scf:  # automatic connection
    logging.info("Connected!")

    scf.cf.param.set_value('stabilizer.estimator', 2)  # set KF as estimator

    crazy.reset_estimator(sc_s.cf)

    height_drone = 0

    poslog = LogConfig(name='Position', period_in_ms=10)
    poslog.add_variable('stateEstimate.x', 'float')
    poslog.add_variable('stateEstimate.y', 'float')
    poslog.add_variable('stateEstimate.z', 'float')
    scf.cf.log.add_config(poslog)
    orlog = LogConfig(name='Stabilizer', period_in_ms=10)
    orlog.add_variable('stabilizer.roll', 'float')
    orlog.add_variable('stabilizer.pitch', 'float')
    orlog.add_variable('stabilizer.yaw', 'float')
    scf.cf.log.add_config(orlog)

    poslog.start()
    orlog.start()

    # AUTO TAKE-OFF to sc_v.DEFAULT_HEIGHT!
    # Class used for the position control during the take-off phase:
    # take-off automatic when context created using "with"
    with MotionCommander(scf, sc_v.DEFAULT_HEIGHT) as mc:
        logging.info('===============Take-Off!================')

        logging.info("==================Right===================")
        # Getting a frame
        logging.info("Getting a frame during flight...")
        try:
            sc_v.got_frame = 0
            while not sc_v.got_frame:
                sc_v.got_frame = sc_s.vicon.GetFrame()
                sc_v.new_frame = sc_s.vicon.GetFrameNumber()
                logging.debug("Fetched! Pulled frame number: %d"
                              if sc_v.got_frame
                              else "Vicon is not streaming!",
                              sc_v.new_frame)
        except ViconDataStream.DataStreamException as exc:
            logging.error("Error while getting a frame in the core! "
                          "--> %s", exc)
            exit("Error while getting a frame in the core: " + str(exc))

        logging.info("Getting drone position...")
        try:
            drone_trans = sc_s.vicon.GetSegmentGlobalTranslation(
                sc_v.drone, 'Crazyflie')
            logging.info("Drone position: " + drone_trans)
        except ViconDataStream.DataStreamException as exc:
            logging.error("Error while getting drone position:!"
                          " --> %s", exc)
            exit("Error while getting drone position:" + str(exc))

        logging.info("Getting drone rotation matrix from Vicon...")
        # all in Global Vicon -> RPY in global
        try:
            mat_gl = sc_s.vicon. \
                GetSegmentGlobalRotationMatrix(sc_v.drone, 'Crazyflie')
            mat_gl = np.array(mat_gl[0])
            logging.info("Drone rotmat: " + mat_gl)
        except ViconDataStream.DataStreamException as exc:
            logging.error("Error while getting the rotation matrix! "
                          "--> %s", exc)
            exit("Error while getting the rotation matrix: "
                 + str(exc))

        mc.right(0.3)

        logging.info("==================Forward===================")
        # Getting a frame
        logging.info("Getting a frame during flight...")
        try:
            sc_v.got_frame = 0
            while not sc_v.got_frame:
                sc_v.got_frame = sc_s.vicon.GetFrame()
                sc_v.new_frame = sc_s.vicon.GetFrameNumber()
                logging.debug("Fetched! Pulled frame number: %d"
                              if sc_v.got_frame
                              else "Vicon is not streaming!",
                              sc_v.new_frame)
        except ViconDataStream.DataStreamException as exc:
            logging.error("Error while getting a frame in the core! "
                          "--> %s", exc)
            exit("Error while getting a frame in the core: " + str(exc))

        logging.info("Getting drone position...")
        try:
            drone_trans = sc_s.vicon.GetSegmentGlobalTranslation(
                sc_v.drone, 'Crazyflie')
            logging.info("Drone position: " + drone_trans)
        except ViconDataStream.DataStreamException as exc:
            logging.error("Error while getting drone position:!"
                          " --> %s", exc)
            exit("Error while getting drone position:" + str(exc))

        logging.info("Getting drone rotation matrix from Vicon...")
        # all in Global Vicon -> RPY in global
        try:
            mat_gl = sc_s.vicon. \
                GetSegmentGlobalRotationMatrix(sc_v.drone, 'Crazyflie')
            mat_gl = np.array(mat_gl[0])
            logging.info("Drone rotmat: " + mat_gl)
        except ViconDataStream.DataStreamException as exc:
            logging.error("Error while getting the rotation matrix! "
                          "--> %s", exc)
            exit("Error while getting the rotation matrix: "
                 + str(exc))

        mc.forward(0.3)

        logging.info("==================Left===================")
        # Getting a frame
        logging.info("Getting a frame during flight...")
        try:
            sc_v.got_frame = 0
            while not sc_v.got_frame:
                sc_v.got_frame = sc_s.vicon.GetFrame()
                sc_v.new_frame = sc_s.vicon.GetFrameNumber()
                logging.debug("Fetched! Pulled frame number: %d"
                              if sc_v.got_frame
                              else "Vicon is not streaming!",
                              sc_v.new_frame)
        except ViconDataStream.DataStreamException as exc:
            logging.error("Error while getting a frame in the core! "
                          "--> %s", exc)
            exit("Error while getting a frame in the core: " + str(exc))

        logging.info("Getting drone position...")
        try:
            drone_trans = sc_s.vicon.GetSegmentGlobalTranslation(
                sc_v.drone, 'Crazyflie')
            logging.info("Drone position: " + drone_trans)
        except ViconDataStream.DataStreamException as exc:
            logging.error("Error while getting drone position:!"
                          " --> %s", exc)
            exit("Error while getting drone position:" + str(exc))

        logging.info("Getting drone rotation matrix from Vicon...")
        # all in Global Vicon -> RPY in global
        try:
            mat_gl = sc_s.vicon. \
                GetSegmentGlobalRotationMatrix(sc_v.drone, 'Crazyflie')
            mat_gl = np.array(mat_gl[0])
            logging.info("Drone rotmat: " + mat_gl)
        except ViconDataStream.DataStreamException as exc:
            logging.error("Error while getting the rotation matrix! "
                          "--> %s", exc)
            exit("Error while getting the rotation matrix: "
                 + str(exc))

        mc.left(0.3)

        logging.info("==================Backwards===================")
        # Getting a frame
        logging.info("Getting a frame during flight...")
        try:
            sc_v.got_frame = 0
            while not sc_v.got_frame:
                sc_v.got_frame = sc_s.vicon.GetFrame()
                sc_v.new_frame = sc_s.vicon.GetFrameNumber()
                logging.debug("Fetched! Pulled frame number: %d"
                              if sc_v.got_frame
                              else "Vicon is not streaming!",
                              sc_v.new_frame)
        except ViconDataStream.DataStreamException as exc:
            logging.error("Error while getting a frame in the core! "
                          "--> %s", exc)
            exit("Error while getting a frame in the core: " + str(exc))

        logging.info("Getting drone position...")
        try:
            drone_trans = sc_s.vicon.GetSegmentGlobalTranslation(
                sc_v.drone, 'Crazyflie')
            logging.info("Drone position: " + drone_trans)
        except ViconDataStream.DataStreamException as exc:
            logging.error("Error while getting drone position:!"
                          " --> %s", exc)
            exit("Error while getting drone position:" + str(exc))

        logging.info("Getting drone rotation matrix from Vicon...")
        # all in Global Vicon -> RPY in global
        try:
            mat_gl = sc_s.vicon. \
                GetSegmentGlobalRotationMatrix(sc_v.drone, 'Crazyflie')
            mat_gl = np.array(mat_gl[0])
            logging.info("Drone rotmat: " + mat_gl)
        except ViconDataStream.DataStreamException as exc:
            logging.error("Error while getting the rotation matrix! "
                          "--> %s", exc)
            exit("Error while getting the rotation matrix: "
                 + str(exc))

        mc.back(0.3)

        logging.info("==================Final===================")
        # Getting a frame
        logging.info("Getting a frame during flight...")
        try:
            sc_v.got_frame = 0
            while not sc_v.got_frame:
                sc_v.got_frame = sc_s.vicon.GetFrame()
                sc_v.new_frame = sc_s.vicon.GetFrameNumber()
                logging.debug("Fetched! Pulled frame number: %d"
                              if sc_v.got_frame
                              else "Vicon is not streaming!",
                              sc_v.new_frame)
        except ViconDataStream.DataStreamException as exc:
            logging.error("Error while getting a frame in the core! "
                          "--> %s", exc)
            exit("Error while getting a frame in the core: " + str(exc))

        logging.info("Getting drone position...")
        try:
            drone_trans = sc_s.vicon.GetSegmentGlobalTranslation(
                sc_v.drone, 'Crazyflie')
            logging.info("Drone position: " + drone_trans)
        except ViconDataStream.DataStreamException as exc:
            logging.error("Error while getting drone position:!"
                          " --> %s", exc)
            exit("Error while getting drone position:" + str(exc))

        logging.info("Getting drone rotation matrix from Vicon...")
        # all in Global Vicon -> RPY in global
        try:
            mat_gl = sc_s.vicon. \
                GetSegmentGlobalRotationMatrix(sc_v.drone, 'Crazyflie')
            mat_gl = np.array(mat_gl[0])
            logging.info("Drone rotmat: " + mat_gl)
        except ViconDataStream.DataStreamException as exc:
            logging.error("Error while getting the rotation matrix! "
                          "--> %s", exc)
            exit("Error while getting the rotation matrix: "
                 + str(exc))

        poslog.stop()
        orlog.stop()
