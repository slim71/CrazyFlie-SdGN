import logging
import threading
import time
import numpy as np
import math
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.position_hl_commander import PositionHlCommander
from own_module import crazyfun as crazy, script_setup as sc_s, \
    script_variables as sc_v


with SyncCrazyflie(sc_v.uri, sc_s.cf) as scf:
    scf.cf.param.set_value('stabilizer.estimator', 2)  # set KF as estimator
    scf.cf.param.set_value('commander.enHighLevel', '1')

    crazy.reset_estimator(scf)

    crazy.int_matlab.write("% x y z r p yw")
    datalog = crazy.datalog(scf)
    datalog.start()

    time.sleep(0.5)

    crazy.set_matlab.write("% set_x set_y set_z")
    crazy.wand_matlab.write("% wand_x wand_y wand_z")

    est_thread = threading.Thread(target=crazy.repeat_fun,
                                  args=(crazy.vicon2drone_period,
                                        crazy.pose_sending, scf))
    wand_thread = threading.Thread(target=crazy.repeat_fun,
                                   args=(crazy.wand_period,
                                         crazy.wand_sending))
    est_thread.start()
    wand_thread.start()

    setpoint = (0, 0, 0)

    with PositionHlCommander(
            scf,
            x=0.0, y=0.0, z=0.0,
            default_velocity=0.2,
            default_height=0.5,
            controller=PositionHlCommander.CONTROLLER_PID) as pc:

        logging.info("Take-off!")

        lowPowerCount = 0
        while lowPowerCount < 5:

            # REPULSIVE POTENTIAL FROM WAND TO DRONE
            wand2drone_dist = (sc_v.drone_pos[0] - sc_v.wand_pos[0],
                               sc_v.drone_pos[1] - sc_v.wand_pos[1])
            dir = math.atan2(wand2drone_dist[1], wand2drone_dist[0])
            w2d_norm = 1/np.linalg.norm(wand2drone_dist)*0.05
            repulsive_pot = (w2d_norm*math.cos(dir), w2d_norm*math.sin(dir))

            # ATTRACTIVE POTENTIAL FROM SETPOINT TO DRONE
            drone_set_dist = (setpoint[0] - sc_v.drone_pos[0],
                              setpoint[1] - sc_v.drone_pos[1])
            attractive_pot = (drone_set_dist[0]*0.7, drone_set_dist[1]*0.7)

            pc.go_to(sc_v.drone_pos[0] + repulsive_pot[0] + attractive_pot[0],
                     sc_v.drone_pos[1] + repulsive_pot[1] + attractive_pot[1])

            crazy.set_matlab.write(sc_v.drone_pos[0] + repulsive_pot[0] + attractive_pot[0],
                                   sc_v.drone_pos[1] + repulsive_pot[1] + attractive_pot[1],
                                   setpoint[2])

            if crazy.battery == 3:
                lowPowerCount = lowPowerCount + 1
            else:
                lowPowerCount = 0

    # unreachable due to the while, but left out for completeness
    datalog.stop()
