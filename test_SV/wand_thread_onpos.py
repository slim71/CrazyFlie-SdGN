import logging
import threading
import time
import numpy as np
from numpy import linalg
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.position_hl_commander import PositionHlCommander
from own_module import crazyfun as crazy, script_setup as sc_s, \
    script_variables as sc_v


# Warn user
print("Starting thread position capture in 5s!")
time.sleep(5)

crazy.wand_matlab.write("% wand_x wand_y wand_z")
wand_thread = threading.Thread(target=crazy.repeat_fun,
                               args=(crazy.wand_period,
                                     crazy.wand_sending))
wand_thread.daemon = True
wand_thread.start()

precedent = sc_v.wand_pos  # At this point this is already initialized
equal_pos = 0              # counter of wand position equal to precedent
while crazy.run:
    time.sleep(crazy.wand_period)
    print(linalg.norm(np.array(precedent) - np.array(sc_v.wand_pos)))
    if equal_pos >= crazy.max_equal_pos:
        crazy.run = False
    if linalg.norm(np.array(precedent) - np.array(sc_v.wand_pos)) <= crazy.pos_limit:  # [m]
        equal_pos += 1
    precedent = sc_v.wand_pos  # update precedent

print("10s before taking off! Prepare to take a video!")
time.sleep(10)

with SyncCrazyflie(sc_v.uri, sc_s.cf) as scf:

    scf.cf.param.set_value('stabilizer.estimator', 2)  # set KF as estimator
    scf.cf.param.set_value('commander.enHighLevel', '1')

    crazy.reset_estimator(scf)

    crazy.int_matlab.write("% x y z qx qy qz qw")
    crazy.set_matlab.write("% set_x set_y set_z")

    datalog = crazy.datalog(scf)
    datalog.start()

    time.sleep(0.5)

    crazy.run = True
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

        logging.info("Take-off!")

        lowPowerCount = 0

        while lowPowerCount < 5:
            crazy.wand_setpoint = crazy.wand_matlab.read_point()
            print(crazy.wand_setpoint)

            pc.go_to(float(crazy.wand_setpoint[0]),
                     float(crazy.wand_setpoint[1]),
                     float(crazy.wand_setpoint[2]))

            crazy.set_matlab.write(crazy.wand_setpoint[0],
                                   crazy.wand_setpoint[1],
                                   crazy.wand_setpoint[2])

            logging.info("Current battery state is %d", crazy.battery)

            if crazy.battery == 3:
                lowPowerCount = lowPowerCount + 1
            else:
                lowPowerCount = 0

    print("LANDING!")
    datalog.stop()
