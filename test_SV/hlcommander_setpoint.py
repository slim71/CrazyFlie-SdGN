import logging
import threading
import time
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.position_hl_commander import PositionHlCommander
from cflib.crazyflie import HighLevelCommander
from timeit import default_timer as timer
from own_module import crazyfun as crazy, script_setup as sc_s, \
    script_variables as sc_v
from own_module import sequences as seq


with SyncCrazyflie(sc_v.uri, sc_s.cf) as scf:
    scf.cf.param.set_value('stabilizer.estimator', 2)  # set KF as estimator
    scf.cf.param.set_value('commander.enHighLevel', '1')

    crazy.reset_estimator(scf)
    crazy.wait_for_position_estimator(scf)

    datalog = crazy.datalog(scf)
    datalog.start()

    time.sleep(0.5)

    crazy.int_matlab.write("% x y z qx qy qz qw")
    crazy.set_matlab.write("% set_x set_y set_z")

    est_thread = threading.Thread(target=crazy.repeat_fun,
                                  args=(crazy.vicon2drone_period,
                                        crazy.pose_sending, scf))
    est_thread.start()

    pc = HighLevelCommander(scf.cf)

    logging.info("Take-off!")
    for i in range(20):
        pc.go_to(0,0,0.5,0, 2, relative=False)
        time.sleep(0.1)

    time.sleep(3)
    for setpoint in seq.other_square:
        for i in range(20):
            time.sleep(0.1)
            pc.go_to(setpoint[0], setpoint[1], setpoint[2],
                 0, 2, relative=False)
    datalog.stop()
