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


def move_square():
        i = 0
        while i < 4:
            time.sleep(1)
            mc.forward(0.2)
            time.sleep(1)
            mc.turn_left(90)
            time.sleep(1)
            i = i + 1

def log_pos_callback(timestamp, data, logconf):
    print(data)
    global position_estimate
    position_estimate[0] = data['stateEstimate.x']
    position_estimate[1] = data['stateEstimate.y']

if __name__ == '__main__':
    cflib.crtp.init_drivers(enable_debug_driver=False)

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        #scf.cf.param.set_value('stabilizer.estimator', 2)

        logconf = LogConfig(name='Position', period_in_ms=100)
        logconf.add_variable('stateEstimate.x', 'float')
        logconf.add_variable('stateEstimate.y', 'float')
        scf.cf.log.add_config(logconf)
        logconf.data_received_cb.add_callback(log_pos_callback)

        logconf.start()

        with MotionCommander(scf, DEFAULT_HEIGHT) as mc:
            move_square()

        #with MotionCommander(scf, DEFAULT_HEIGHT) as mc:
        #    time.sleep(1)
        #    print("sleep done")
        #    mc.forward(0.3)
        #    print("forward done")
        #    mc.back(0.3)
        #    print("back done")
        #    time.sleep(1)
        logconf.stop()

