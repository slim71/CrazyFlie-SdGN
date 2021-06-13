import numpy as np

# Vicon connection settings
VICON_IP = "192.168.0.2"  # Set the IP of the Vicon server to connect to
VICON_PORT = "801"  # Set the port of the Vicon server to connect to
drone = "Crazyflie2"  # Set the "Vicon name" of the object linked to the drone

# Set the "Vicon-name" of the Wand
Wand = "Active Wand v2 (Origin Tracking)"

# Crazyflie address
uri = 'radio://0/80/2M'

# The height the drone has to reach at the end of the take-off.
DEFAULT_HEIGHT = 0.5  # [m]

# Utility flags
got_frame = 0
last_frame = 0
new_frame = 0
vicon_conn = 0
attempts = 10

# Used in the generation of the position reference for the drone
drone_or = np.mat([0.0, 0.0, 0.0, 0.0])
drone_pos = np.array([0.0, 0.0, 0.0])
last_wand_pos = np.array([0.0, 0.0, 0.0])
wand_pos_m = np.array([0.0, 0.0, 0.0])
wand_pos = np.array([0.0, 0.0, 0.0])
wand_trans = np.array([0.0, 0.0, 0.0])
