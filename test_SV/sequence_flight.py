import logging
import time
from own_module import crazyfun as crazy
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
import script_variables as sc_v
import script_setup as sc_s
import sequences as seq
from vicon_dssdk import ViconDataStream

with SyncCrazyflie(sc_v.uri, sc_s.cf) as scf:
    scf.cf.param.set_value('stabilizer.estimator', 2)  # set KF as estimator

    datalog = crazy.datalog(scf)
    datalog.start()

    with MotionCommander(scf, sc_v.DEFAULT_HEIGHT) as mc:
        logging.info("Take-off!")

        to_fly = seq.square

        for point in to_fly:
            for i in range(10):  # 10Hz like this (?)
                try:
                    sc_s.vicon.GetFrame()
                except ViconDataStream.DataStreamException as exc:
                    logging.error("Error while getting a frame in the core! "
                                  "--> %s", exc)
                sc_v.drone_pos = sc_s.vicon.\
                    GetSegmentGlobalTranslation(sc_v.drone, sc_v.drone)[0]

                scf.cf.extpos.send_extpos(sc_v.drone_pos[0],
                                          sc_v.drone_pos[1],
                                          sc_v.drone_pos[2])
                scf.cf.commander.send_position_setpoint(point[0],
                                                        point[1],
                                                        point[2],
                                                        point[3])
                time.sleep(0.1)

    datalog.stop()
