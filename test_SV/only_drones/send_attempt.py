import logging
import time
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
import own_module.script_variables as sc_v
import script_setup as sc_s

with SyncCrazyflie(sc_v.uri, sc_s.cf) as scf:
    scf.cf.param.set_value('stabilizer.estimator', 2)  # set KF as estimator

    with MotionCommander(scf, sc_v.DEFAULT_HEIGHT) as mc:
        logging.info("Take-off!")

        logging.info("Will try to fly in a square")

        logging.debug("Sending 0.5m in x direction")
        for i in range(20):
            scf.cf.commander.send_position_setpoint(
                0.5, 0, sc_v.DEFAULT_HEIGHT, 0)
            # Apparently, every time people send setpoints, they also use this
            time.sleep(0.1)

        # logging.debug("Sending 0.5m in y direction")
        # for i in range(10):
        #     sc_s.cf.commander.send_position_setpoint(
        #          0.5, 0, sc_v.DEFAULT_HEIGHT, 90)
        #     time.sleep(0.5)

        logging.debug("Sending -0.5m in x direction")
        for i in range(10):
            scf.cf.commander.send_position_setpoint(
                -0.5, 0, sc_v.DEFAULT_HEIGHT, 0)
            time.sleep(0.5)

        # logging.debug("Sending -0.5m in y direction")
        # for i in range(10):
        #     sc_s.cf.commander.send_position_setpoint(
        #          -0.5, 0, sc_v.DEFAULT_HEIGHT, 270)
        #     time.sleep(0.5)

# va a scatti così com'è
