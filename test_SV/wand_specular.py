import logging
import time
from cflib.positioning.position_hl_commander import PositionHlCommander
from own_module import crazyfun as crazy
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
import script_variables as sc_v
import script_setup as sc_s
import sequences as seq
from vicon_dssdk import ViconDataStream

with SyncCrazyflie(sc_v.uri, sc_s.cf) as scf:
    scf.cf.param.set_value('stabilizer.estimator', 2)  # set KF as estimator
    scf.cf.param.set_value('commander.enHighLevel', '1')
    crazy.reset_estimator(scf)
    # crazy.wait_for_position_estimator(scf)

    datalog = crazy.datalog(scf)
    datalog.start()

    time.sleep(0.5)

    with PositionHlCommander(
            scf,
            x=0.0, y=0.0, z=0.0,
            default_velocity=0.3,
            default_height=0.5,
            controller=PositionHlCommander.CONTROLLER_PID) as pc:

        logging.info("Take-off!")

        # crazy.matlab_print("% SQUARE")
        crazy.matlab_print("% x y z  "
                           "qx qy qz qw")

        while 1:

            # get a new frame
            try:
                sc_s.vicon.GetFrame()
            except ViconDataStream.DataStreamException as exc:
                logging.error("Error while getting a frame in the core! "
                              "--> %s", str(exc))

            # get current position and orientation in Vicon
            sc_v.drone_pos = sc_s.vicon. \
                GetSegmentGlobalTranslation(sc_v.drone, sc_v.drone)[0]
            sc_v.drone_or = sc_s.vicon. \
                GetSegmentGlobalRotationQuaternion(sc_v.drone,
                                                   sc_v.drone)[0]

            sc_v.drone_pos = (float(sc_v.drone_pos[0] / 1000),
                              float(sc_v.drone_pos[1] / 1000),
                              float(sc_v.drone_pos[2] / 1000))

            scf.cf.extpos.send_extpose(sc_v.drone_pos[0],
                                       sc_v.drone_pos[1],
                                       sc_v.drone_pos[2],
                                       sc_v.drone_or[0],
                                       sc_v.drone_or[1],
                                       sc_v.drone_or[2],
                                       sc_v.drone_or[3])

            sc_v.wand_pos = sc_s.vicon. \
                GetSegmentGlobalTranslation(sc_v.Wand, sc_v.Wand)[0]
            sc_v.wand_pos_m = (float(sc_v.wand_pos[0] / 1000),
                               float(sc_v.wand_pos[1] / 1000),
                               float(sc_v.wand_pos[2] / 1000))

            pc.go_to(-sc_v.Wand_Translation[0],
                     -sc_v.Wand_Translation[1],
                     sc_v.Wand_Translation[2])

            crazy.matlab_print(sc_v.drone_pos[0], sc_v.drone_pos[1],
                               sc_v.drone_pos[2],
                               sc_v.drone_or[0], sc_v.drone_or[1],
                               sc_v.drone_or[2], sc_v.drone_or[2])

    datalog.stop()
