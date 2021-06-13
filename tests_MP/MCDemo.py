import logging
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander

URI = 'radio://0/80/2M'

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)


if __name__ == '__main__':
    # Initialize the low-level drivers
    cflib.crtp.init_drivers()

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        # We take off when the commander is created
        with MotionCommander(scf) as mc:
            time.sleep(1)

            # There is a set of functions that move a specific distance
            # We can move in all directions
            mc.forward(0.8)
            mc.back(0.8)
            time.sleep(1)

            mc.up(0.5)
            mc.down(0.5)
            time.sleep(1)

            # We can also set the velocity
            mc.right(0.5, velocity=0.8)
            time.sleep(1)
            mc.left(0.5, velocity=0.4)
            time.sleep(1)

            # We can do circles or parts of circles
            mc.circle_right(0.5, velocity=0.5, angle_degrees=180)

            # Or turn
            mc.turn_left(90)
            time.sleep(1)

            # We can move along a line in 3D space
            mc.move_distance(-1, 0.0, 0.5, velocity=0.6)
            time.sleep(1)

            # There is also a set of functions that start a motion. The
            # Crazyflie will keep on going until it gets a new command.

            mc.start_left(velocity=0.5)
            # The motion is started and we can do other stuff, printing for
            # instance
            for _ in range(5):
                print('Doing other work')
                time.sleep(0.2)

            # And we can stop
            mc.stop()
