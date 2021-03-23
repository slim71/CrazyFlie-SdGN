import logging
import time
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from own_module import crazyfun as crazy
import script_variables as sc_v
import script_setup as sc_s

with SyncCrazyflie(sc_v.uri, sc_s.cf) as scf:

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

    with MotionCommander(scf, sc_v.DEFAULT_HEIGHT) as mc:

        logging.info("Take-off!")
        time.sleep(1)

    poselog.stop()
    datalog.stop()
