from __future__ import print_function
from vicon_dssdk import ViconDataStream
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig
from cflib.positioning.motion_commander import MotionCommander
import time
import logging
import script_variables as sc_v
import script_setup as sc_s
from own_module import crazyfun as crazy

# controllare log ultima esecuzione: è sceso quando non doveva dopo essere
# andato a sx

# Class used to start the synchronization with the drone
with SyncCrazyflie(sc_v.uri, sc_s.cf) as scf:  # automatic connection
    logging.info("Connected!")

    scf.cf.param.set_value('stabilizer.estimator', 2)  # set KF as estimator

    crazy.reset_estimator(sc_s.cf)
    crazy.wait_for_position_estimator(sc_s.cf)

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
    # Class used for the setpoint control during the take-off phase:
    # take-off automatic when context created using "with"
    with MotionCommander(scf, sc_v.DEFAULT_HEIGHT) as mc:
        logging.info('===============Take-Off!================')
        logging.info("Will try a square movement with just one frame")

        logging.debug("At first")

        logging.info("Getting a frame...")
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
            logging.error("Error while getting a frame! --> %s", exc)
            exit("Error while getting a frame: " + str(exc))

        logging.info("Getting drone setpoint...")
        try:
            sc_v.drone_pos = sc_s.vicon.\
                GetSegmentGlobalTranslation(sc_v.drone, 'Crazyflie2')
            logging.info("Done.")
        except ViconDataStream.DataStreamException as exc:
            logging.error("Error while getting Drone setpoint! --> %s", exc)
            exit("Error while getting Drone setpoint: " + str(exc))
        sc_v.drone_pos = sc_v.drone_pos[0]
        # Maybe it has to be converted in meters?
        # sc_v.drone_pos_m = np.array([float(sc_v.drone_pos[0]) / 1000,
        #                              float(sc_v.drone_pos[1]) / 1000,
        #                              float(sc_v.drone_pos[2]) / 1000])
        time.sleep(0.1)  # seems to be better to use this every time?

        sc_s.cf.extpos.send_extpos(sc_v.drone_pos[0],
                                   sc_v.drone_pos[1],
                                   sc_v.drone_pos[2])

        mc.forward(0.5)
        time.sleep(0.1)

        logging.debug("1---After first movement")

        mc.left(0.5)
        time.sleep(0.1)

        logging.debug("2---After second movement")

        mc.back(0.5)
        time.sleep(0.1)

        logging.debug("3---After third movement")

        mc.right(0.5)
        time.sleep(0.1)

        logging.debug("4---After fourth movement")

        logging.debug("===Landing!===")

time.sleep(5)

with SyncCrazyflie(sc_v.uri, sc_s.cf) as scf:  # automatic connection
    logging.info("Connected!")

    scf.cf.param.set_value('stabilizer.estimator', 2)  # set KF as estimator

    crazy.reset_estimator(sc_s.cf)
    crazy.wait_for_position_estimator(sc_s.cf)

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
    # Class used for the setpoint control during the take-off phase:
    # take-off automatic when context created using "with"
    with MotionCommander(scf, sc_v.DEFAULT_HEIGHT) as mc:
        logging.info('===============Take-Off!================')
        logging.info("Will try a square movement with a frame each time")

        logging.debug("At first")

        logging.info("Getting a frame...")
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
            logging.error("Error while getting a frame! --> %s", exc)
            exit("Error while getting a frame: " + str(exc))

        logging.info("Getting drone setpoint...")
        try:
            sc_v.drone_pos = sc_s.vicon.\
                GetSegmentGlobalTranslation(sc_v.drone, 'Crazyflie2')
            logging.info("Done.")
        except ViconDataStream.DataStreamException as exc:
            logging.error("Error while getting Drone setpoint! --> %s", exc)
            exit("Error while getting Drone setpoint: " + str(exc))
        sc_v.drone_pos = sc_v.drone_pos[0]
        # Maybe it has to be converted in meters?
        # sc_v.drone_pos_m = np.array([float(sc_v.drone_pos[0]) / 1000,
        #                              float(sc_v.drone_pos[1]) / 1000,
        #                              float(sc_v.drone_pos[2]) / 1000])
        time.sleep(0.1)  # seems to be better to use this every time?

        sc_s.cf.extpos.send_extpos(sc_v.drone_pos[0],
                                   sc_v.drone_pos[1],
                                   sc_v.drone_pos[2])

        mc.forward(0.5)
        time.sleep(0.1)

        logging.debug("1---After first movement")

        logging.info("Getting a frame...")
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
            logging.error("Error while getting a frame! --> %s", exc)
            exit("Error while getting a frame: " + str(exc))

        logging.info("Getting drone setpoint...")
        try:
            sc_v.drone_pos = sc_s.vicon. \
                GetSegmentGlobalTranslation(sc_v.drone, 'Crazyflie2')
            logging.info("Done.")
        except ViconDataStream.DataStreamException as exc:
            logging.error("Error while getting Drone setpoint! --> %s", exc)
            exit("Error while getting Drone setpoint: " + str(exc))
        sc_v.drone_pos = sc_v.drone_pos[0]
        # Maybe it has to be converted in meters?
        # sc_v.drone_pos_m = np.array([float(sc_v.drone_pos[0]) / 1000,
        #                              float(sc_v.drone_pos[1]) / 1000,
        #                              float(sc_v.drone_pos[2]) / 1000])
        time.sleep(0.1)  # seems to be better to use this every time?

        sc_s.cf.extpos.send_extpos(sc_v.drone_pos[0],
                                   sc_v.drone_pos[1],
                                   sc_v.drone_pos[2])

        mc.left(0.5)
        time.sleep(0.1)

        logging.debug("2---After second movement")

        logging.info("Getting a frame...")
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
            logging.error("Error while getting a frame! --> %s", exc)
            exit("Error while getting a frame: " + str(exc))

        logging.info("Getting drone setpoint...")
        try:
            sc_v.drone_pos = sc_s.vicon. \
                GetSegmentGlobalTranslation(sc_v.drone, 'Crazyflie2')
            logging.info("Done.")
        except ViconDataStream.DataStreamException as exc:
            logging.error("Error while getting Drone setpoint! --> %s", exc)
            exit("Error while getting Drone setpoint: " + str(exc))
        sc_v.drone_pos = sc_v.drone_pos[0]
        # Maybe it has to be converted in meters?
        # sc_v.drone_pos_m = np.array([float(sc_v.drone_pos[0]) / 1000,
        #                              float(sc_v.drone_pos[1]) / 1000,
        #                              float(sc_v.drone_pos[2]) / 1000])
        time.sleep(0.1)  # seems to be better to use this every time?

        sc_s.cf.extpos.send_extpos(sc_v.drone_pos[0],
                                   sc_v.drone_pos[1],
                                   sc_v.drone_pos[2])

        mc.back(0.5)
        time.sleep(0.1)

        logging.debug("3---After third movement")

        logging.info("Getting a frame...")
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
            logging.error("Error while getting a frame! --> %s", exc)
            exit("Error while getting a frame: " + str(exc))

        logging.info("Getting drone setpoint...")
        try:
            sc_v.drone_pos = sc_s.vicon. \
                GetSegmentGlobalTranslation(sc_v.drone, 'Crazyflie2')
            logging.info("Done.")
        except ViconDataStream.DataStreamException as exc:
            logging.error("Error while getting Drone setpoint! --> %s", exc)
            exit("Error while getting Drone setpoint: " + str(exc))
        sc_v.drone_pos = sc_v.drone_pos[0]
        # Maybe it has to be converted in meters?
        # sc_v.drone_pos_m = np.array([float(sc_v.drone_pos[0]) / 1000,
        #                              float(sc_v.drone_pos[1]) / 1000,
        #                              float(sc_v.drone_pos[2]) / 1000])
        time.sleep(0.1)  # seems to be better to use this every time?

        sc_s.cf.extpos.send_extpos(sc_v.drone_pos[0],
                                   sc_v.drone_pos[1],
                                   sc_v.drone_pos[2])

        mc.right(0.5)
        time.sleep(0.1)

        logging.debug("4---After fourth movement")

        time.sleep(3)

        logging.debug("===Landing!===")
