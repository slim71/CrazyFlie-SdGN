import logging
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander

URI = 'radio://0/80/2M'
DEFAULT_HEIGHT = 0.3

is_deck_attached = False
position_estimate = [0, 0]

logging.basicConfig(level=logging.ERROR)


if __name__ == '__main__':
    cflib.crtp.init_drivers(enable_debug_driver=False)

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        # scf.cf.param.set_value('stabilizer.estimator', 2)

        logconf = LogConfig(name='Position', period_in_ms=100)
        logconf.add_variable('stateEstimate.x', 'float')
        logconf.add_variable('stateEstimate.y', 'float')
        scf.cf.log.add_config(logconf)

        logconf.start()

        with MotionCommander(scf, DEFAULT_HEIGHT) as mc:
            logging.info("===right===")
            mc.right(0.3)
            logging.info("===forward===")
            mc.forward(0.3)
            logging.info("===left===")
            mc.left(0.3)
            logging.info("===backward===")
            mc.backward(0.3)

        logconf.stop()
