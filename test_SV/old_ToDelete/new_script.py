from __future__ import print_function
from vicon_dssdk import ViconDataStream
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
import numpy as np
import logging
from own_module import crazyfun as crazy
import time
import script_variables as sc_v
import script_setup as sc_s

# Class used to start the synchronization with the drone
with SyncCrazyflie(sc_v.uri, sc_s.cf) as scf:  # automatic connection
    logging.info("Connected!")

    height_drone = 0

    # AUTO TAKE-OFF to sc_v.DEFAULT_HEIGHT!
    # Class used for the position control during the take-off phase:
    # take-off automatic when context created using "with"
    with MotionCommander(scf, sc_v.DEFAULT_HEIGHT) as mc:
        logging.info('===============Take-Off!================')

        # Test of the standard Motion Commander
        mc.forward(0.3)
        logging.debug("forward done")
        mc.back(0.3)
        logging.debug("back done")
        time.sleep(5)

        logging.info("==================Core===================")
        while 1:

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

            # Absolute position of the Wand in [mm], then converted in [m]
            logging.info("Getting Wand position...")
            try:
                wand_p = sc_s.vicon.GetSegmentGlobalTranslation(sc_v.Wand,
                                                                'Root')
                logging.info("Done.")
            except ViconDataStream.DataStreamException as exc:
                logging.error("Error while getting Wand position! --> %s",
                              exc)
                exit("Error while getting Wand position: " + str(exc))
            wand_trans_mm = sc_v.wand_pos[0]
            sc_v.wand_pos_m = np.array([float(wand_trans_mm[0]) / 1000,
                                        float(wand_trans_mm[1]) / 1000,
                                        float(wand_trans_mm[2]) / 1000])

            # Assuming the Wand sends (0, 0, 0) when turned off,
            # this is considered as an error
            if not all(sc_v.wand_pos_m):  # sc_v.wand_trans_m == [0, 0, 0]:
                sc_v.CONSEC_LOSSES += 1
                logging.debug("Fault! Consecutive losses: %d",
                              sc_v.CONSEC_LOSSES)

                # Should apply the homogeneous matrix, but it's null anyway
                sc_v.Wand_Translation = np.array([0, 0, 0])
                sc_v.last_wand_pos = sc_v.wand_pos_m

                if sc_v.CONSEC_LOSSES == crazy.MAX_LOSS:
                    # LANDING PHASE: We received MAX_LOSS consecutive
                    # null positions of the Wand, so we decide to land
                    logging.error("LANDING! %d consecutive "
                                  "null positions received.",
                                  crazy.MAX_LOSS)
                    exit("Too many position losses: drone landing...")
            else:
                logging.info("!!Normal cycle execution!!")

                # reset counter
                sc_v.CONSEC_LOSSES = 0

                # We get the actual position of drone expressed in
                # the Body frame and send it to the Kalman Filter
                logging.info("Getting drone position...")
                try:
                    drone_trans = sc_s.vicon.GetSegmentGlobalTranslation(
                        sc_v.drone, 'Crazyflie')  # TODO: not wrt Root?
                    logging.info("Done.")
                except ViconDataStream.DataStreamException as exc:
                    logging.error("Error while getting drone position:!"
                                  " --> %s", exc)
                    exit("Error while getting drone position:" + str(exc))
                drone_trans_mm = drone_trans[0]
                sc_v.drone_pos_m = np.array([float(drone_trans_mm[0]) / 1000,
                                             float(drone_trans_mm[1]) / 1000,
                                             float(drone_trans_mm[2]) / 1000]
                                            )

                logging.info("Getting drone rotation matrix from Vicon...")
                # all in Global Vicon -> RPY in global
                try:
                    mat_gl = sc_s.vicon.\
                        GetSegmentGlobalRotationMatrix(sc_v.drone, 'Crazyflie')
                    mat_gl = np.array(mat_gl[0])
                except ViconDataStream.DataStreamException as exc:
                    logging.error("Error while getting the rotation matrix! "
                                  "--> %s", exc)
                    exit("Error while getting the rotation matrix: "
                         + str(exc))

                logging.info("Getting drone Euler angles from Vicon...")
                try:
                    eul = sc_s.vicon.\
                        GetSegmentGlobalRotationEulerXYZ(sc_v.drone,
                                                         'Crazyflie')
                    eul = np.array(eul[0])
                except ViconDataStream.DataStreamException as exc:
                    logging.error("Error while getting Euler angles! "
                                  "--> %s", exc)
                    exit("Error  while getting Euler angles: " + str(exc))

                # Summary of the situazione
                last_dp = sc_v.last_drone_pos
                last_dr = sc_v.last_drone_ref
                last_wp = sc_v.last_wand_pos

                sc_v.last_wand_pos = sc_v.wand_pos_m
                sc_v.last_drone_ref += sc_v.Wand_Translation
                sc_v.last_drone_pos = sc_v.drone_pos_m

            logging.info("Printing available data...")
            logging.debug("Wand current position (in Vicon system): %s",
                          str(sc_v.wand_pos_m))
            logging.debug("Wand last position (in Vicon system): %s",
                          str(last_wp))
            logging.debug("Wand_Translation (in Vicon system): %s",
                          str(sc_v.Wand_Translation))
            logging.debug("Drone current position (in Vicon system): %s",
                          str(sc_v.drone_pos_m))
            logging.debug("Drone last position (in Vicon system): %s",
                          str(last_dp))
            logging.debug("Drone current reference point (in Vicon system):"
                          " %s", str(sc_v.last_drone_ref))
            logging.debug("Drone last reference point (in Vicon system): %s",
                          str(last_dr))
            logging.debug("Vicon rotation matrix: %s ",
                          str(mat_gl))

    # Automatic land exiting the "with" environment!
