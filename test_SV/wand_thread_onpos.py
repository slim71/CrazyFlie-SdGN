import logging
import threading
import time
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
equal_pos = 0
while crazy.run:
    precedent = sc_v.wand_pos  # update precedent
    if equal_pos >= crazy.max_equal_pos:
        crazy.run = False
    if abs(precedent - sc_v.wand_pos) <= crazy.pos_limit:  # [m]
        equal_pos += 1


with SyncCrazyflie(sc_v.uri, sc_s.cf) as scf:

    scf.cf.param.set_value('stabilizer.estimator', 2)  # set KF as estimator
    scf.cf.param.set_value('commander.enHighLevel', '1')

    crazy.reset_estimator(scf)

    crazy.int_matlab.write("% x y z qx qy qz qw")
    crazy.set_matlab.write("% set_x set_y set_z")

    datalog = crazy.datalog(scf)
    datalog.start()

    time.sleep(0.5)

    est_thread = threading.Thread(target=crazy.repeat_fun,
                                  args=(crazy.vicon2drone_period,
                                        crazy.pose_sending, scf))
    setpoint_thread = threading.Thread(target=crazy.repeat_fun,
                                       args=(crazy.wand_period,
                                             crazy.set_wand_track))

    # Set threads as daemon: this way they will terminate as soon as
    # the main program terminates
    est_thread.daemon = True
    setpoint_thread.daemon = True

    est_thread.start()
    setpoint_thread.start()

    with PositionHlCommander(
            scf,
            x=0.0, y=0.0, z=0.0,
            default_velocity=0.3,
            default_height=0.5,
            controller=PositionHlCommander.CONTROLLER_PID) as pc:

        logging.info("Take-off!")

        lowPowerCount = 0

        while lowPowerCount < 5:
            pc.go_to(crazy.wand_setpoint[0],
                     crazy.wand_setpoint[1],
                     crazy.wand_setpoint[2])

            crazy.set_matlab.write(crazy.wand_setpoint[0],
                                   crazy.wand_setpoint[1],
                                   crazy.wand_setpoint[2])

            logging.info("Current battery state is %d", crazy.battery)

            if crazy.battery <= 3:
                lowPowerCount = lowPowerCount + 1
            else:
                lowPowerCount = 0

    print("LANDING!")
    datalog.stop()
