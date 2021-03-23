# sequenza in terna vicon convertita poi in globale

import logging
import time
from datetime import datetime
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from own_module.log_estimation_manager import LogEstimationManager
from own_module.vicon_manager import ViconManager
import math

URI = 'radio://0/80/2M'
DEFAULT_HEIGHT = 0.5
RADIUS = 0.5

position_estimate = [0, 0, 0]
attitude_estimate = [0, 0, 0]

# -----------------------------------------------------------------------------
# ----------------------------------DEBUG--------------------------------------
# -----------------------------------------------------------------------------
filename = "../other_or_old/Logs/LogFile_CrazyFlie_NOFLAG" \
		   + datetime.now().strftime("%Y%m%d_%H%M%S")

logname = filename + ".log"
filename = filename + ".txt"
logging.basicConfig(filename=logname,
					level=logging.DEBUG,
					filemode='a',
					format="%(asctime)s [%(levelname)s]: %(message)s")

# -----------------------------------------------------------------------------
# sequence in vicon frame
# sequence_V = [(0, 0, DEFAULT_HEIGHT, 0)]
# theta = 0.0
# for j in range(16):
# 	theta = theta + 2 * math.pi / 16
# 	sequence_V.append((RADIUS * math.cos(theta), RADIUS * math.sin(theta), DEFAULT_HEIGHT,
# 					   (theta+math.pi)*180/math.pi))
# sequence_V.append((0, 0, 0.3, math.pi*180/math.pi))
sequence_V = [
    (0, 0, DEFAULT_HEIGHT, 90),
    (0.5, 0, DEFAULT_HEIGHT, 0),
    (0.5, 0, DEFAULT_HEIGHT, 90),
]

if __name__ == '__main__':

	with LogEstimationManager() as log_est:
		with SyncCrazyflie(URI) as scf:
			cf = scf.cf
			log_est.simple_log_async(cf)
			log_est.reset_estimator(cf)
			vicon = ViconManager()
			time.sleep(0.5)
			V2G_dist, V2G_rot = vicon.get_global_frame()
			for setpoint_V in sequence_V:
				Btransl_G, Brot_G = vicon.change2G_coord((setpoint_V[0], setpoint_V[1], setpoint_V[2]),
														 vicon.yaw2quat(setpoint_V[3]),
														 V2G_dist, V2G_rot)
				setpoint_G = (Btransl_G[0], Btransl_G[1], Btransl_G[2],
							  vicon.quat2yaw(Brot_G))
				for i in range(10):
					pos_drone, quat_drone = vicon.get_drone_pose(V2G_dist, V2G_rot)
					cf.extpos.send_extpos(pos_drone[0], pos_drone[1],
										   pos_drone[2])
					# cf.extpos.send_extpose(pos_drone[0], pos_drone[1], pos_drone[2],
					# 					   quat_drone[0], quat_drone[1], quat_drone[2], quat_drone[3])
					cf.commander.send_position_setpoint(setpoint_G[0], setpoint_G[1], setpoint_G[2], setpoint_G[3])
					time.sleep(0.5)

			cf.commander.send_stop_setpoint()
			time.sleep(0.5)
