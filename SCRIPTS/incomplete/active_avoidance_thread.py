import time
import threading
import logging
import math
import numpy as np
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.position_hl_commander import PositionHlCommander
from own_module import crazyfun as crazy, script_setup as sc_s, \
    script_variables as sc_v


# Class used to start the synchronization with the drone
with SyncCrazyflie(sc_v.uri, sc_s.cf) as scf:  # automatic connection
    scf.cf.param.set_value('stabilizer.estimator', 2)  # set KF as estimator
    scf.cf.param.set_value('commander.enHighLevel', '1')

    crazy.reset_estimator(scf)

    crazy.int_matlab.write("% x y z qx qy qz qw")
    datalog = crazy.datalog(scf)
    datalog.start()

    time.sleep(0.5)

    crazy.set_matlab.write("% set_x set_y set_z")

    est_thread = threading.Thread(target=crazy.repeat_fun,
                                  args=(crazy.vicon2drone_period,
                                        crazy.pose_sending, scf))

    # Set threads as daemon: this way they will terminate as soon as
    # the main program terminates
    est_thread.daemon = True

    est_thread.start()

    with PositionHlCommander(
            scf,
            x=0.0, y=0.0, z=0.0,
            default_velocity=0.3,
            default_height=0.5,
            controller=PositionHlCommander.CONTROLLER_PID) as pc:

        obs_thread = threading.Thread(target=crazy.repeat_fun,
                                      args=(crazy.obstacle_period,
                                            crazy.check_obstable, pc))
        obs_thread.daemon = True
        obs_thread.start()

        logging.info('===============Take-Off!================')

        lowPowerCount = 0

        while lowPowerCount < 5:
            logging.info("Loop rpound")

    print("LANDING!")
    datalog.stop()
