import logging
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
import own_module.script_variables as sc_v
import script_setup as sc_s
from own_module import crazyfun as crazy

with SyncCrazyflie(sc_v.uri, sc_s.cf) as scf:
    scf.cf.param.set_value('stabilizer.estimator', 2)  # set KF as estimator

    crazy.wait_for_position_estimator(sc_s.cf)

    poslog = LogConfig(name='Position', period_in_ms=100)
    poslog.add_variable('stateEstimate.x', 'float')
    poslog.add_variable('stateEstimate.y', 'float')
    poslog.add_variable('stateEstimate.z', 'float')
    scf.cf.log.add_config(poslog)
    orlog = LogConfig(name='Stabilizer', period_in_ms=100)
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
        mc.back(0.3)

    poslog.stop()
    orlog.stop()
