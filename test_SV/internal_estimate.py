import logging
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
import test_SV.script_variables as sc_v
import script_setup as sc_s

with SyncCrazyflie(sc_v.uri, cf=Crazyflie(rw_cache='./cache')) as scf:
    # scf.cf.param.set_value('stabilizer.estimator', 2)

    poslog = LogConfig(name='Position', period_in_ms=10)
    poslog.add_variable('stateEstimate.x', 'float')
    poslog.add_variable('stateEstimate.y', 'float')
    poslog.add_variable('stateEstimate.z', 'float')
    scf.cf.log.add_config(poslog)
    orlog = LogConfig(name='Stabilizer', period_in_ms=10)
    orlog.add_variable('stabilizer.roll', 'float')
    orlog.add_variable('stabilizer.pitch', 'float')
    orlog.add_variable('stabilizer.yaw', 'float')
    scf.cf.log.add_config(orlog)

    poslog.start()
    orlog.start()

    with MotionCommander(scf, sc_v.DEFAULT_HEIGHT) as mc:
        logging.info("===right===")
        mc.right(0.3)
        logging.info("===forward===")
        mc.forward(0.3)
        logging.info("===left===")
        mc.left(0.3)
        logging.info("===backward===")
        mc.backward(0.3)

    poslog.stop()
    orlog.stop()
