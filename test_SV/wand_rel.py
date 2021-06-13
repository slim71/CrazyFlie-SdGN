import logging
import threading
import time
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.position_hl_commander import PositionHlCommander
from own_module import crazyfun as crazy, script_setup as sc_s, \
    script_variables as sc_v


with SyncCrazyflie(sc_v.uri, sc_s.cf) as scf:
    scf.cf.param.set_value('stabilizer.estimator', 2)  # set KF as estimator
    scf.cf.param.set_value('commander.enHighLevel', '1')

    crazy.reset_estimator(scf)
    # crazy.wait_for_position_estimator(scf)  # included in reset_estimator

    datalog = crazy.datalog(scf)
    datalog.start()

    time.sleep(0.5)

    crazy.int_matlab.write("% x y z qx qy qz qw")
    crazy.set_matlab.write("% set_x set_y set_z")
    crazy.wand_matlab.write("% wand_x wand_y wand_z")

    est_thread = threading.Thread(target=crazy.repeat_fun,
                                  args=(crazy.vicon2drone_period,
                                        crazy.pose_sending, scf))
    est_thread.start()

    with PositionHlCommander(
            scf,
            x=0.0, y=0.0, z=0.0,
            default_velocity=0.3,
            default_height=0.5,
            controller=PositionHlCommander.CONTROLLER_PID) as pc:

        logging.info("Take-off!")

        sc_v.wand_pos = sc_s.vicon. \
            GetSegmentGlobalTranslation(sc_v.Wand, "Root")[0]  # sc_v.Wand)[0]
        sc_v.wand_pos_m = (float(sc_v.wand_pos[0] / 1000),
                           float(sc_v.wand_pos[1] / 1000),
                           float(sc_v.wand_pos[2] / 1000))
        sc_v.last_wand_pos = (sc_v.wand_pos_m[0],
                              sc_v.wand_pos_m[1],
                              sc_v.wand_pos_m[2])
        logging.debug("wand reference: %s", str(sc_v.last_wand_pos))

        while 1:
            time.sleep(1)
            logging.debug("wand last: %s", str(sc_v.last_wand_pos))
            # new frame not needed since the other thread gets one every 0.1s?
            sc_v.wand_pos = sc_s.vicon.\
                GetSegmentGlobalTranslation(sc_v.Wand, "Root")[0]  # sc_v.Wand)[0]
            sc_v.wand_pos_m = (float(sc_v.wand_pos[0] / 1000),
                               float(sc_v.wand_pos[1] / 1000),
                               float(sc_v.wand_pos[2] / 1000))
            logging.debug("wand position: %s", str(sc_v.wand_pos_m))

            sc_v.wand_trans = (sc_v.wand_pos_m[0] - sc_v.last_wand_pos[0],
                               sc_v.wand_pos_m[1] - sc_v.last_wand_pos[1],
                               sc_v.wand_pos_m[2] - sc_v.last_wand_pos[2])
            logging.debug("wand translation: %s", str(sc_v.wand_trans))

            setpoint = (sc_v.drone_pos[0] + sc_v.wand_trans[0],
                        sc_v.drone_pos[1] + sc_v.wand_trans[1],
                        sc_v.drone_pos[2] + sc_v.wand_trans[2])

            pc.go_to(setpoint[0], setpoint[1], setpoint[2])
            logging.debug("sepoint sent: %s", str(setpoint))

            sc_v.last_wand_pos = (sc_v.wand_pos_m[0],
                                  sc_v.wand_pos_m[1],
                                  sc_v.wand_pos_m[2])

            crazy.set_matlab.write(
                sc_v.drone_pos[0] + sc_v.wand_trans[0],
                sc_v.drone_pos[1] + sc_v.wand_trans[1],
                sc_v.drone_pos[2] + sc_v.wand_trans[2])

    # unreachable due to the while, but left out for completeness
    datalog.stop()
