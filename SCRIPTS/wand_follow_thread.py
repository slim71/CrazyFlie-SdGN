import logging
import threading
import time
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.position_hl_commander import PositionHlCommander
from own_module import crazyfun as crazy, script_setup as sc_s, \
    script_variables as sc_v


with SyncCrazyflie(sc_v.uri, sc_s.cf) as scf:
    scf.cf.param.set_value('stabilizer.estimator', 2)  # set KF as estimator
    scf.cf.param.set_value('commander.enHighLevel', '1')

    crazy.reset_estimator(scf)

    crazy.int_matlab.write("% x y z qx qy qz qw")
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
    # Set threads as daemon: this way they will terminate as soon as
    # the main program terminates
    est_thread.daemon = True
    wand_thread.daemon = True

    est_thread.start()
    wand_thread.start()

    with PositionHlCommander(
            scf,
            x=0.0, y=0.0, z=0.0,
            default_velocity=0.3,
            default_height=0.5,
            controller=PositionHlCommander.CONTROLLER_PID) as pc:

        logging.info("Take-off!")

        lowPowerCount = 0

        while lowPowerCount < 5:
            # wand_pos is updated by an ad-hoc thread
            pc.go_to(sc_v.wand_pos[0]-crazy.safety_offset,
                     sc_v.wand_pos[1]-crazy.safety_offset,
                     sc_v.wand_pos[2])

            crazy.set_matlab.write(sc_v.wand_pos[0]-crazy.safety_offset,
                                   sc_v.wand_pos[1]-crazy.safety_offset,
                                   sc_v.wand_pos[2])

            logging.info("Current battery state is %d", crazy.battery)

            if crazy.battery <= 3:
                lowPowerCount = lowPowerCount + 1
            else:
                lowPowerCount = 0

    print("LANDING!")
    datalog.stop()
