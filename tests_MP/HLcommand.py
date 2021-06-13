import cflib.crtp
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.position_hl_commander import PositionHlCommander
from own_module.log_estimation_manager import LogEstimationManager
from own_module.vicon_manager import ViconManager
import os
import time
from datetime import datetime
from pathlib import Path

URI = 'radio://0/80/2M'

sequence_V = [
	(0, 0, 0.3),
	(0.2, 0.2, 0.5),
	(-0.2, 0.2, 0.5),
	(-0.2, -0.2, 0.5),
	(0.2, -0.2, 0.5),
	(0, 0, 0.3)
]

if __name__ == '__main__':
	data_file = "./data_logs/" + datetime.now().strftime("__%Y%m%d_%H%M")
	data_file = data_file + ".txt"
	ff = os.path.normpath(os.path.join(Path(__file__).parent.absolute(),
									   data_file))

	with LogEstimationManager() as log_est:
		with SyncCrazyflie(URI) as scf:
			cf = scf.cf
			log_est.simple_log_async(cf)
			log_est.reset_estimator(cf)
			vicon = ViconManager()
			time.sleep(0.5)
			init_pos, V2G_rot = vicon.get_drone_pose()
			with PositionHlCommander(
					scf,
					x=init_pos[0], y=init_pos[1], z=init_pos[2],
					default_velocity=0.3,
					default_height=0.5,
					controller=PositionHlCommander.CONTROLLER_PID) as pc:
				with open(ff, 'a') as descr:
					for setpoint in sequence_V:
						for i in range(20):
							pos_drone, quat_drone = vicon.get_drone_pose(V2G_rot)
							pos_est, att_est = log_est.get_estimation()
							cf.extpos.send_extpose(pos_drone[0], pos_drone[1], pos_drone[2],
												   quat_drone[0], quat_drone[1], quat_drone[2], quat_drone[3])
							print(pos_drone[0], pos_drone[1], pos_drone[2],
								  quat_drone[0], quat_drone[1], quat_drone[2], quat_drone[3],
								  file=descr)
							print(pos_est[0], pos_est[1], pos_est[2],
								  att_est[0], att_est[1], att_est[2],
								  file=descr)

							pc.go_to(setpoint[0], setpoint[1], setpoint[2])
							time.sleep(0.1)

