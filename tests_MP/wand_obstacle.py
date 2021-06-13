import logging
import threading
import time
import numpy as np
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

    setpoint = (0.9, -0.9, 0.5)

    with PositionHlCommander(
            scf,
            x=0.0, y=0.0, z=0.0,
            default_velocity=0.2,
            default_height=0.5,
            controller=PositionHlCommander.CONTROLLER_PID) as pc:

        logging.info("Take-off!")

        while 1:
            # wand_pos is updated by an ad-hoc thread
            wand_drone_dist = (sc_v.drone_pos[0] - sc_v.wand_pos[0],
                               sc_v.drone_pos[1] - sc_v.wand_pos[1])
            wand_drone_norm = wand_drone_dist/np.linalg.norm(wand_drone_dist)
            repulsive_pot = wand_drone_norm*0.4

            drone_set_dist = (setpoint[0] - sc_v.drone_pos[0],
                              setpoint[1] - sc_v.drone_pos[1])
            drone_set_norm = drone_set_dist/np.linalg.norm(drone_set_dist)
            attractive_pot = drone_set_norm*0.4

            pc.go_to(repulsive_pot[0] + attractive_pot[0],
                     repulsive_pot[1] + attractive_pot[1],
                     setpoint[2])

            crazy.set_matlab.write(repulsive_pot[0] + attractive_pot[0],
                                   repulsive_pot[1] + attractive_pot[1],
                                   setpoint[2])

    # unreachable due to the while, but left out for completeness
    datalog.stop()