import numpy as np

VICON_IP = "192.168.0.2"  # Set the IP of the Vicon server to connect to
VICON_PORT = "801"  # Set the port of the Vicon server to connect to
drone = "Crazyflie2"  # Set the "Vicon name" of the object relative to the drone

# Set the "Vicon-name" of the object relative to the Wand
Wand = "Active Wand v2 (Origin Tracking)"
uri = 'radio://0/80/2M'  # Used for the connection with the drone

WAIT_TIME = 5  # [s]  Used for the management of the iterations
SLEEP_TIME = 0.001  # [s]  Used for the management of the iterations
ITERATIONS = WAIT_TIME / SLEEP_TIME  # Used to manage the iterations

# Security offset
SEC_OFFSET = 0.3  # [m]

# To be subtracted from the "z" component of the last setpoint of the drone
# during each iteration of the landing phase
DELTA_HEIGHT = 0.01  # [m]

# Sum of meters subtracted from the  "z" component of
# the last setpoint of the drone
SUBTRACTED_HEIGHT = 0.01  # [m]


# The height the drone has to reach at the end of the take-off. This can't
# be higher than the "MC_HEIGHT" used in the class of the
# Motion Commander. We suggest to set it at least at 90% of its value.
DEFAULT_HEIGHT = 0.5  # [m]

# This is the default height used by the Motion Commander during take-off.
# It has to be higher than DEFAULT_HEIGHT because we consider the
# take-off phase concluded once the drone reaches DEFAULT_HEIGHT, but this
# can't be always true because the Vicon might observe another value due to
# noise
MC_HEIGHT = 0.8  # [m]

# Number of frames for the Vicon buffer
buffer_size = 1000

got_frame = 0
last_frame = 0
new_frame = 0

# Current number of consecutive losses in the acquisition of the Wand setpoint
CONSEC_LOSSES = 0

# Used to update the yaw angle of the drone during the experiment
last_gamma = 0

# Used in the generation of the setpoint reference for the drone
last_trans = np.array([0.0, 0.0, 0.0])
last_drone_pos = np.array([0.0, 0.0, 0.0])
last_drone_ref = np.array([0.0, 0.0, 0.0])
drone_pos_m = np.array([0.0, 0.0, 0.0])
drone_pos = np.array([0.0, 0.0, 0.0])
last_wand_pos = np.array([0.0, 0.0, 0.0])
wand_pos_m = np.array([0.0, 0.0, 0.0])
wand_pos = np.array([0.0, 0.0, 0.0])
Wand_Translation = np.array([0.0, 0.0, 0.0])


is_deck_attached = False
position_estimate = [0, 0]
