from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
import script_variables as sc_v
import script_setup as sc_s
import logging
import numpy as np

sequence = [
    (0.5, 0, sc_v.DEFAULT_HEIGHT, 0),
    (0, 0, sc_v.DEFAULT_HEIGHT, 0),
    (-0.5, 0, sc_v.DEFAULT_HEIGHT, 0)
]


with SyncCrazyflie(sc_v.uri, sc_s.cf) as scf:  # automatic connection
    scf.cf.param.set_value('stabilizer.estimator', 2)  # set KF as estimator

    with MotionCommander(scf, sc_v.DEFAULT_HEIGHT) as mc:
        logging.info('===============Take-Off!================')

        sc_v.drone_pos = sc_s.vicon.\
            GetSegmentGlobalTranslation(sc_v.drone, sc_v.drone)[0]
        sc_v.drone_pos = np.array(sc_v.drone_pos, dnmin=2)

        sc_v.drone_or = sc_s.vicon.\
            GetSegmentGlobalRotationMatrix(sc_v.drone, sc_v.drone)[0]
        sc_v.drone_or = np.mat(sc_v.drone_or)

        rototrasl = np.concatenate(
            (np.concatenate((sc_v.drone_or, sc_v.drone_or.T), axis=1),
             np.array([[0, 0, 0, 1]])), axis=0)

        point = np.array(sequence[0], ndmin=2)
