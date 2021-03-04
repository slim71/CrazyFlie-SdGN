import logging
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
import test_SV.script_variables as sc_v

with SyncCrazyflie(sc_v.uri, cf=Crazyflie(rw_cache='./cache')) as scf:

    with MotionCommander(scf, sc_v.DEFAULT_HEIGHT) as mc:

