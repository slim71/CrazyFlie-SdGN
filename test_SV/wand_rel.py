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
    crazy.wait_for_position_estimator(scf)

    datalog = crazy.datalog(scf)
    datalog.start()

    time.sleep(0.5)

    crazy.matlab_print("% x y z qx qy qz qw")

    est_thread = threading.Thread(target=crazy.repeat_fun,
                                  args=(0.1, crazy.pose_sending, scf)
                                  )

    with PositionHlCommander(
            scf,
            x=0.0, y=0.0, z=0.0,
            default_velocity=0.3,
            default_height=0.5,
            controller=PositionHlCommander.CONTROLLER_PID) as pc:

        logging.info("Take-off!")

        while 1:
            # new frame not needed since the other thread gets one every 0.1s?
            sc_v.wand_pos = sc_s.vicon.\
                GetSegmentGlobalTranslation(sc_v.Wand, sc_v.Wand)[0]
            sc_v.wand_pos_m = (float(sc_v.wand_pos[0] / 1000),
                               float(sc_v.wand_pos[1] / 1000),
                               float(sc_v.wand_pos[2] / 1000))

            sc_v.wand_trans = (sc_v.wand_pos_m[0] - sc_v.last_wand_pos[0],
                               sc_v.wand_pos_m[1] - sc_v.last_wand_pos[1],
                               sc_v.wand_pos_m[2] - sc_v.last_wand_pos[2])

            pc.go_to(sc_v.drone_pos[0] + sc_v.wand_trans[0] - 0.3,
                     sc_v.drone_pos[1] + sc_v.wand_trans[1] - 0.3,
                     sc_v.drone_pos[2] + sc_v.wand_trans[2])

            sc_v.last_wand_pos = sc_v.wand_pos_m

    # unreachable due to the while, but left out for completeness
    datalog.stop()
