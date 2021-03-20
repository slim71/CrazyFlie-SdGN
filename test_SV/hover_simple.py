import logging
import time
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
<<<<<<< HEAD
from own_module import crazyfun as crazy
=======
>>>>>>> 032fbdc12721e16559a9d53207dbb6d6447f8ca6
import script_variables as sc_v
import script_setup as sc_s

with SyncCrazyflie(sc_v.uri, cf=Crazyflie(rw_cache='./cache')) as scf:

<<<<<<< HEAD
    poselog = LogConfig(name='PoseLog', period_in_ms=1000)
    poselog.add_variable('stateEstimate.x', 'float')
    poselog.add_variable('stateEstimate.y', 'float')
    poselog.add_variable('stateEstimate.z', 'float')
    poselog.add_variable('stabilizer.roll', 'float')
    poselog.add_variable('stabilizer.pitch', 'float')
    poselog.add_variable('stabilizer.yaw', 'float')
    scf.cf.log.add_config(poselog)
    poselog.start()

    datalog = crazy.datalog(scf)
    datalog.start()
=======
    poslog = LogConfig(name='Position', period_in_ms=1000)
    poslog.add_variable('stateEstimate.x', 'float')
    poslog.add_variable('stateEstimate.y', 'float')
    poslog.add_variable('stateEstimate.z', 'float')
    scf.cf.log.add_config(poslog)
    orlog = LogConfig(name='Stabilizer', period_in_ms=1000)
    orlog.add_variable('stabilizer.roll', 'float')
    orlog.add_variable('stabilizer.pitch', 'float')
    orlog.add_variable('stabilizer.yaw', 'float')
    scf.cf.log.add_config(orlog)

    poslog.start()
    orlog.start()
>>>>>>> 032fbdc12721e16559a9d53207dbb6d6447f8ca6

    with MotionCommander(scf, sc_v.DEFAULT_HEIGHT) as mc:

        logging.info("Take-off!")
<<<<<<< HEAD
        time.sleep(10)

    poselog.stop()
    datalog.stop()
=======
        time.sleep(60)

        # If it doesn't work, try this:
        # sc_s.cf.param.set_value("flightmode.althold", "True")
        # it = 0
        # while it < 700:
        #     sc_s.cf.commander.send_setpoint(0, 0, 0, 32767)
        #     sc_s.cf.param.set_value("flightmode.althold", "True")
        #     time.sleep(0.01)
        #     it += 1

    poslog.stop()
    orlog.stop()
>>>>>>> 032fbdc12721e16559a9d53207dbb6d6447f8ca6
