#------------------------------------------------------------------------------------------------------------------------------------------------------------
#----------------------------------IMPORT--------------------------------------------------------------------------------------------------------------------
#------------------------------------------------------------------------------------------------------------------------------------------------------------
from __future__ import print_function
from vicon_dssdk import ViconDataStream
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.commander import Commander
from cflib.positioning.motion_commander import MotionCommander
from cflib.positioning.position_hl_commander import PositionHlCommander
from cflib.crazyflie.extpos import Extpos
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.crtp.crtpstack import CRTPPacket
from cflib.crtp.crtpstack import CRTPPort
from datetime import datetime
import numpy as np
import cflib.crtp
import math
import cflib.crazyflie
import cflib.bootloader
import cflib.drivers
import cflib.positioning
import cflib.utils
import threading
import argparse
import logging
import time
import struct
#------------------------------------------------------------------------------------------------------------------------------------------------------------
#----------------------------------SET UP--------------------------------------------------------------------------------------------------------------------
#------------------------------------------------------------------------------------------------------------------------------------------------------------

VICON_IP = "192.168.0.2"                    #Set with ip of the vicon server you want to connect to
VICON_PORT = "801"                          #Set with port of the vicon server you want to connect to
Drone = "Crazyflie"                         #Set with the "vicon-name" of the object relative to the drone
Wand = "Active Wand v2 (Origin Tracking)"   #Set with the "vicon-name" of the object relative to the Wand
uri = 'radio://0/80/2M'                     #Used for the connection with the drone

MAKE_LOG_FILE = 1                           #Set to 0 if you don't want to obtain the log file after the execution. Otherwise set to 1. The log file can be used by the matlab file "plot.m"
KALMAN_INCLUDE_QUATERNION = 0               #Set to 1 if you want to send orientation toghether with the position to the kalman filter.
ACTIVATE_KALMAN_DURING_TAKEOFF = 0          #We strictly recommend to don't send Kalman updates during the take off if you use the MotionCommander (so manintain it set to zero)
LOG_TEST_WITH_DISACTIVATED_THRUSTER = 0     #Set to 1 if you want to do a test without make the Drone fly. In that case, in order to get as more log-data as possible, we suggest to set the other flags to these values:
                                                    #LOG_TEST_THRUSTER_DISACTIVATED = 1 (Obviously)
                                                    #MAKE_LOG_FILE = 1
                                                    #KALMAN_INCLUDE_QUATERNION = 1
                                                    #ACTIVATE_KALMAN_DURING_TAKEOFF = 1 (In this case there isn't a proper "take off phase" so if you don't enable this flag you never send information to kalman)

#------------------------------------------------------------------------------------------------------------------------------------------------------------
#----------------------------------VARIABLES-----------------------------------------------------------------------------------------------------------------
#------------------------------------------------------------------------------------------------------------------------------------------------------------

T_prec = np.array([0, 0, 0])                #Used in the generation of the position reference for the drone
W_T_prec = np.array([0, 0, 0])              #Used in the generation of the position reference for the drone
take_off = 1                                #It will be set to 0 after the take off
last_position_send = np.array([0, 0, 0])    #It is updated in each iteration and it will be used as the starting point for the landing phase.
last_gamma = 0                              #Used for update the yaw angle of the Drone during the experiment
height_drone = 0                            #Current height of the Drone. It is used only in the take off phase.
quaternion = np.array([0, 0, 0, 0])         #Used to store the orientation of the drone
last_quaternion = np.array([0, 0, 0, 0])    #Used in the updating of the orientation of the drone to the Kalman filter

OFFSET=0.3 #[m]                             #Security offset
MAX_LOSS = 10                               #Max number of consecutive loss allowed during acquisition of the position of the Wand or Drone
CONSECUTIVE_LOSS = 0                        #Current number of consecutive loss in the acquisition of the wand position
DELTA_HEIGHT=0.01 #[m]                      #[m] to be subtracted from the "z" component of the last position of the drone during each iteration of the landing phase
SUBTRACTED_HEIGHT = 0.01 #[m]               #Sum of [m] subtracted from the  "z" component of the last position of the drone
DEFAULT_HEIGHT = 0.5 #[m]                   #The height that the drone has to reach at the end of the take off. This can't be higher than the "MOTION_COMMANDER_DEFAULT_HEIGHT" used in the class of the Motion Commander. We suggest to set it at least at 90% of its value.
MOTION_COMMANDER_DEFAULT_HEIGHT = 0.8 #[m]  #This is the default height used by the Motion Commander during the take off. If it has to be higher than DEFAULT_HEIGHT because we consider the take off phase conlcued once the drone reachs DEFAULT_HEIGHT, but this can't be true if they are equal and maybe the Vicon observe a minor value (due to noise)

log_pos_x = 0.0                             #Used to store the parameters of the Drone's log table (TOC)
log_pos_y = 0.0                             #Used to store the parameters of the Drone's log table (TOC)
log_pos_z = 0.0                             #Used to store the parameters of the Drone's log table (TOC)
log_roll = 0.0                              #Used to store the parameters of the Drone's log table (TOC)
log_pitch = 0.0                             #Used to store the parameters of the Drone's log table (TOC)
log_yaw = 0.0                               #Used to store the parameters of the Drone's log table (TOC)

#------------------------------------------------------------------------------------------------------------------------------------------------------------
#-------------------------------FUNCTIONS--------------------------------------------------------------------------------------------------------------------
#------------------------------------------------------------------------------------------------------------------------------------------------------------

#This function resets the Kalman filter before flying.
def reset_estimator(cf):
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')

#This is the function used for consulting the log table in the function "simple_log_async"
def log_stab_callback(timestamp, data, logconf):
    #print('[%d][%s]: %s' % (timestamp, logconf.name, data))
    global log_pos_x
    global log_pos_y
    global log_pos_z
    global log_roll
    global log_pitch
    global log_yaw
    log_pos_x = data['stateEstimate.x']
    log_pos_y = data['stateEstimate.y']
    log_pos_z = data['stateEstimate.z']
    log_roll = data['stabilizer.roll']
    log_pitch = data['stabilizer.pitch']
    log_yaw = data['stabilizer.yaw']

#This function is used in "config_logging" and it configures the log table of the Drone adding the callback for the desired parameters.
def simple_log_async(scf, logconf):
    cf = scf.cf
    cf.log.add_config(logconf)
    logconf.data_received_cb.add_callback(log_stab_callback)

#It is used to prepare the input argument of the "simple_log_async" and to call it.
def config_logging(scf):
    lg_stab = LogConfig(name='Stabilizer', period_in_ms=10)
    lg_stab.add_variable('stateEstimate.x', 'float')
    lg_stab.add_variable('stateEstimate.y', 'float')
    lg_stab.add_variable('stateEstimate.z', 'float')
    lg_stab.add_variable('stabilizer.roll', 'float')
    lg_stab.add_variable('stabilizer.pitch', 'float')
    lg_stab.add_variable('stabilizer.yaw', 'float')
    simple_log_async(scf, lg_stab)
    return lg_stab

#This function gets the position of the Drone from the Vicon client (Tracker).
#It is used only for the first value of position because it is the only one we need
#to define the transaltion between the Navigation System (defined when the Drone
#is turned on) and the Vicon System (fixed in the center of the flight room).
def get_First_Position(client, Drone):

    D_T_meters=np.array([0, 0, 0])
    iterations= 0

    #It can happen that we continue receving the value [0,0,0] for the position for motre than once.
    #This menas that something wrong is happening with the Vicon Tracking so, in order to avoid errors,
    #we count the times we receive [0,0,0] and if it overcomes MAX_LOSS we stop the experiment.
    while(D_T_meters[0] == 0 and D_T_meters[1] == 0 and D_T_meters[2] == 0):
        #This call is necessary before every time we want to call some "GetSegment..()"
        a=client.GetFrame()
        b=client.GetFrameNumber()
        D_T_tuple = client.GetSegmentGlobalTranslation( Drone, Drone )
        #We are interested only in the first part of this structure:
        D_T_millimeters= D_T_tuple[0]
        #Absolute position of the Drone converted from [mm] to [m]:
        D_T_meters = np.array([float(D_T_millimeters[0])/1000 , float(D_T_millimeters[1])/1000, float(D_T_millimeters[2])/1000])

        if(iterations > MAX_LOSS):
            print("STOP EXPERIMENT: initial position of the Drone is exactly [0,0,0] for ", MAX_LOSS, " consecutive times. Please try with another initial position and restart the experiment.")
            exit()
        else:
            iterations = iterations + 1

    return D_T_meters

#This function creates the homogeneous rotation matrix used for the conversions between the Navigation System and the Vicon System.
#The matrix is composed by:
# - Rotation: we assume always null values for roll and pitch angles so the rotation is expressed with a 3x3 matrix on the z-axis with yaw angle "gamma"
# - Translation: it is the initial position of the Drone. It is rotated with the previous matrix.
def create_Matrix_Rotation(client, Drone, D_T_meters, last_gamma,KALMAN_INCLUDE_QUATERNION):

    if KALMAN_INCLUDE_QUATERNION:
    #In this case we don't need to take in consideration the orientation of the drone
    #in the conversion betwwen frames because this will be Kalman Filter concern
        gamma = 0
        last_gamma = 0

    else:
    #Otherwise the K-F doesn't receive updates of the yaw angle from us, so we take it in consideration in the Matrix
    #in order to correctly convert between frames.

        #We get the orientation of the Drone from which we want to get the Yaw angle
        Euler_XYZ = client.GetSegmentGlobalRotationEulerXYZ(Drone, Drone)
        #We are interested only in the first part of the structure:
        Euler_radians = Euler_XYZ[0]

        #In case we receive tha value [0,0,0], that means an error with the Vicon Tracker, we continue using the previuos value of the yaw angle.
        if(Euler_radians[0] == 0 and Euler_radians[1] == 0 and Euler_radians[2] == 0 ):
            gamma = last_gamma
        else:
            gamma = Euler_radians[2]
            last_gamma = gamma

    #roll angle:
    alpha = 0
    #pitch angle:
    beta = 0

    #We build the homogeneous rotation matrix that will convert the positions expressed in the Vicon reference system
    #in positions expressed in the Body frame of the Drone
    R_x = np.array([[1 , 0, 0], [0, math.cos(alpha), -math.sin(alpha)], [0, math.sin(alpha), math.cos(alpha)]])
    R_y = np.array([[math.cos(beta), 0, math.sin(beta)], [0, 1, 0], [-math.sin(beta), 0, math.cos(beta)]])
    R_z = np.array([[math.cos(gamma), -math.sin(gamma), 0], [math.sin(gamma), math.cos(gamma), 0], [0, 0, 1]])
    Matrix_Rotation_xy = np.dot(R_x, R_y)
    Matrix_Rotation_xyz = np.dot(Matrix_Rotation_xy, R_z)
    Matrix_Rotation = np.transpose(Matrix_Rotation_xyz)
    Matrix_homogeneous = np.array([[Matrix_Rotation[0][0], Matrix_Rotation[0][1], Matrix_Rotation[0][2], -np.dot(Matrix_Rotation, np.transpose(D_T_meters))[0]], [Matrix_Rotation[1][0], Matrix_Rotation[1][1], Matrix_Rotation[1][2], -np.dot(Matrix_Rotation, np.transpose(D_T_meters))[1]], [Matrix_Rotation[2][0], Matrix_Rotation[2][1], Matrix_Rotation[2][2], -np.dot(Matrix_Rotation, np.transpose(D_T_meters))[2]], [0, 0, 0, 1]])

    #In case we update the last value of the yaw angle (not null value) we update returning it toghether with the matrix:
    return Matrix_homogeneous,last_gamma


#This function implements the landing phase that starts at the end of the experiment:
def landing(last_position_send,SUBTRACTED_HEIGHT,DELTA_HEIGHT,cf,lg_stab,fdesc,MAKE_LOG_FILE):
    #We gradually reduce the height setpoint:
    while(last_position_send[2]-SUBTRACTED_HEIGHT>0):
        cf.commander.send_position_setpoint(last_position_send[0], last_position_send[1], last_position_send[2]-SUBTRACTED_HEIGHT, 0)
        SUBTRACTED_HEIGHT=SUBTRACTED_HEIGHT+DELTA_HEIGHT

    if MAKE_LOG_FILE:
        #close the connection with the log table:
        lg_stab.stop()
        fdesc.close()

    exit()

    return

#------------------------------------------------------------------------------------------------------------------------------------------------------------
#----------------------------------VICON CONNECTION----------------------------------------------------------------------------------------------------------
#------------------------------------------------------------------------------------------------------------------------------------------------------------

#Extract from PyVicon Library:
print("CONNECTING WITH VICON TRACKER...")
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--hostname", "--"+VICON_IP+":"+VICON_PORT)
args = parser.parse_args()

#Create a VICON Client
client = ViconDataStream.Client()

#Connect to the VICON Server running on the same LAN
client.Connect(VICON_IP+":"+VICON_PORT)
print("Connected")

#Check setting the buffer size works
client.SetBufferSize(1000)
print("Buffer created")

#Enable all the data types
client.EnableSegmentData()
client.EnableMarkerData()
client.EnableUnlabeledMarkerData()
client.EnableMarkerRayData()
client.EnableDeviceData()
client.EnableCentroidData()
print("Data types enabled")

# Report whether the data types have been enabled
print( 'Segments', client.IsSegmentDataEnabled() )
print( 'Markers', client.IsMarkerDataEnabled() )
print( 'Unlabeled Markers', client.IsUnlabeledMarkerDataEnabled() )
print( 'Marker Rays', client.IsMarkerRayDataEnabled() )
print( 'Devices', client.IsDeviceDataEnabled() )
print( 'Centroids', client.IsCentroidDataEnabled() )

# Try setting the different stream modes
client.SetStreamMode( ViconDataStream.Client.StreamMode.EClientPull )
print( 'Get Frame Pull', client.GetFrame(), client.GetFrameNumber() )
client.SetStreamMode( ViconDataStream.Client.StreamMode.EClientPullPreFetch )
print( 'Get Frame PreFetch', client.GetFrame(), client.GetFrameNumber() )
client.SetStreamMode( ViconDataStream.Client.StreamMode.EServerPush )
print( 'Get Frame Push', client.GetFrame(), client.GetFrameNumber() )
print( 'Frame Rate', client.GetFrameRate() )

#Show the frame rate from both client and server side
print( 'Frame Rates' )
for frameRateName, frameRateValue in client.GetFrameRates().items():
        print( frameRateName, frameRateValue )

#Refrence system
client.SetAxisMapping( ViconDataStream.Client.AxisMapping.EForward, ViconDataStream.Client.AxisMapping.ELeft, ViconDataStream.Client.AxisMapping.EUp )
xAxis, yAxis, zAxis = client.GetAxisMapping()
print( 'X Axis', xAxis, 'Y Axis', yAxis, 'Z Axis', zAxis )

#------------------------------------------------------------------------------------------------------------------------------------------------------------
#----------------------------------FLYING MODE---------------------------------------------------------------------------------------------------------------
#------------------------------------------------------------------------------------------------------------------------------------------------------------

#DRONE CONNECTION
print('CONNECTING WITH DRONE...')

if MAKE_LOG_FILE:
    #we create the log file (with the correct name) where we print all the variable of interest:
    filename = "LOG/Log_File_CrazyFlie_" + datetime.now().strftime("%Y%m%d_%H%M%S")
    if(LOG_TEST_WITH_DISACTIVATED_THRUSTER):
        filename = filename + "_ThrusterOff_"
    else:
        filename = filename + "_ThrusterOn_"
    if(ACTIVATE_KALMAN_DURING_TAKEOFF):
        filename = filename + "_KalmanDuringTakeOff_"
    else:
        filename = filename + "_NoKalmanTakeOff_"
    if(KALMAN_INCLUDE_QUATERNION):
        filename = filename + "KalmanWithQuaternion"
    else:
        filename = filename + "KalmanWithoutQuaternion"

    fdesc = open(filename  + ".txt","w")

logging.basicConfig(level=logging.ERROR)
cflib.crtp.init_drivers(enable_debug_driver=False)
cf=cflib.crazyflie.Crazyflie()

First_Position = get_First_Position(client, Drone)

#Class used to start the syncronization with the drone:
with SyncCrazyflie(uri, cf) as scf:

    #We prepare and open the connection to address the Log Table:
    lg_stab = config_logging(scf)

    if MAKE_LOG_FILE:
        #We start the communication only if the MAKE_LOG_FILE is required:
        lg_stab.start()

    #!!! Uncomment if we don't use the FlowDeck !!!
    #cf.param.set_value('stabilizer.estimator', '2')

    #We reset the Kalman Filter before start flying:
    reset_estimator(cf)

    if LOG_TEST_WITH_DISACTIVATED_THRUSTER == 0:
    #Thruster activated:

        #TAKE OFF
        #Class used for the postion control during the take-off phase:
        with MotionCommander(scf, MOTION_COMMANDER_DEFAULT_HEIGHT) as mc:

            print('TAKE OFF')

            while(height_drone < DEFAULT_HEIGHT):

                #Get postion of the drone in the Vicon reference System:
                a=client.GetFrame()
                b=client.GetFrameNumber()
                D_T_tuple = client.GetSegmentGlobalTranslation( Drone, Drone )
                D_T_millimeters = D_T_tuple[0]
                D_T_meters = np.array([float(D_T_millimeters[0])/1000 , float(D_T_millimeters[1])/1000, float(D_T_millimeters[2])/1000])
                #Used for store the Drone's position in the Vicon System.
                D_T_vicon = D_T_meters

                #if KALMAN_INCLUDE_QUATERNION:
                #We decide to get the quaternion even in the case KALMAN_INCLUDE_QUATERNION=0 because in this way we have the same "computational cost" with that flag =0 and =1
                quaternion_XYZW = client.GetSegmentGlobalRotationQuaternion(Drone, Drone)
                quaternion = quaternion_XYZW[0]

                if MAKE_LOG_FILE:
                    #Get orientation of the drone from the Vicon:
                    Euler_angles = client.GetSegmentGlobalRotationEulerXYZ(Drone, Drone)
                    Drone_orientation = Euler_angles[0]

                #We convert vectors in homogenous vector and we convert the position in the body frame:
                Matrix_homogeneous, last_gamma = create_Matrix_Rotation(client, Drone, First_Position, last_gamma, KALMAN_INCLUDE_QUATERNION)
                D_T_homogenous = np.array([D_T_meters[0], D_T_meters[1], D_T_meters[2], 1])
                D_T_homogenous = np.dot(Matrix_homogeneous, np.transpose(D_T_homogenous))
                D_T_meters = np.array([D_T_homogenous[0], D_T_homogenous[1], D_T_homogenous[2]])

                if MAKE_LOG_FILE:
                    #We write in the log file with the following format:
                    #       drone's position body frame                         quaternions                                                 drone's position vicon frame                drone's orientation from Vicon                              setpoint body frame             drone's position and orientation from log table
                    print(D_T_meters[0], D_T_meters[1], D_T_meters[2], quaternion[0], quaternion[1], quaternion[2], quaternion[3], D_T_vicon[0], D_T_vicon[1], D_T_vicon[2], Drone_orientation[0], Drone_orientation[1], Drone_orientation[2], 0, 0, DEFAULT_HEIGHT, 0, log_pos_x, log_pos_y, log_pos_z, log_roll, log_pitch, log_yaw , file=fdesc)

                if ACTIVATE_KALMAN_DURING_TAKEOFF:
                #send measures to Kalman filter also during the take off. We suggest to don't do this while you use the motion commander.
                    if KALMAN_INCLUDE_QUATERNION:
                        cf.extpos.send_extpose(D_T_meters[0], D_T_meters[1], D_T_meters[2], quaternion[0], quaternion[1], quaternion[2], quaternion[3])
                    else:
                        cf.extpos.send_extpos(D_T_meters[0], D_T_meters[1], D_T_meters[2])

                #Update the current height of the drone:
                height_drone = D_T_meters[2]

            #In case we receive [0,0,0] from vicon (this means that an error occurred with the tracker) we store the last postion of the drone for using it instead of the "outlier" [0,0,0]
            last_drone_position = np.array([D_T_meters[0], D_T_meters[1], D_T_meters[2]])

            #this is the end of the take off phase
            take_off = 0

            while(1):

                if(take_off==0):
                    try:
                        #We get the Wand's position expressed in the Vicon system and we convert it from [mm] to [m]:
                        a=client.GetFrame()
                        b=client.GetFrameNumber()
                        W_T_tuple = client.GetSegmentGlobalTranslation( Wand, 'Root' )
                        W_T_millimeters= W_T_tuple[0]
                        W_T_meters = np.array([float(W_T_millimeters[0])/1000 , float(W_T_millimeters[1])/1000, float(W_T_millimeters[2])/1000])

                        #if we receive the value [0,0,0] from the vicon means that an error occurred with the Tracker.
                        #So if the error persists for more than MAX_LOSS times we stop the experiment, otherwise
                        #it is considered as a sort of "outlier" and we use the last correct position used insted of it.
                        #The same happens also when we decide to stop the experiment turing off the wand.
                        if(W_T_meters[0]==0 and W_T_meters[1]==0 and W_T_meters[2]==0):

                            CONSECUTIVE_LOSS=CONSECUTIVE_LOSS+1
                            W_T_meters = last_position_send
                            cf.commander.send_position_setpoint(W_T_meters[0], W_T_meters[1], W_T_meters[2], 0)

                            if(CONSECUTIVE_LOSS==MAX_LOSS):
                                #LANDING PHASE:
                                #We received MAX_LOSS consecutive "null position" of the wand so we decide to start landing.
                                print("START LANDING: ", MAX_LOSS, " consecutive null position of the Wand have been received.")
                                landing(last_position_send,SUBTRACTED_HEIGHT,DELTA_HEIGHT,cf,lg_stab,fdesc,MAKE_LOG_FILE)
                        else:

                            CONSECUTIVE_LOSS=0

                            if KALMAN_INCLUDE_QUATERNION:
                            #In this case we don't have to compute agai the matrix for the rotation because it is constant.
                            #The drone knows its current value of the yaw from K-F and we don't have to change the conversion of the coordinates.

                                quaternion_XYZW= client.GetSegmentGlobalRotationQuaternion(Drone, Drone)
                                quaternion = quaternion_XYZW[0]

                                #if we receive from the vicon all quaternions set to 0 we sent the last correct value:
                                if(quaternion[0]==0 and quaternion[1]==0 and quaternion[2]==0 and quaternion[3]==0):
                                    quaternion = last_quaternion
                                else:
                                    last_quaternion = quaternion

                            else:
                                #If we don't send the orientation to the K-F we have to compute the matrix in each iteration because
                                #the drone doen't know its current yaw and we have to rotate in function of the current yaw angle
                                Matrix_homogeneous, last_gamma = create_Matrix_Rotation(client, Drone, First_Position, last_gamma,KALMAN_INCLUDE_QUATERNION)

                            #We pass from a Wand's position expressed in the Vicon frame to a Wand's position expressed
                            #in the Body frame of the Drone before sending it as the new setpoint:
                            a=client.GetFrame()
                            b=client.GetFrameNumber()
                            W_T_homogenous = np.array([W_T_meters[0]+OFFSET, W_T_meters[1]+OFFSET, W_T_meters[2], 1])
                            W_T_homogenous = np.dot(Matrix_homogeneous, np.transpose(W_T_homogenous))
                            W_T_meters = np.array([W_T_homogenous[0], W_T_homogenous[1], W_T_homogenous[2]])

                            #We send the new setpoint:
                            cf.commander.send_position_setpoint(W_T_meters[0], W_T_meters[1], W_T_meters[2], 0)
                            last_position_send = W_T_meters

                            if MAKE_LOG_FILE:
                                #We will use this information when we write in the log file:
                                Euler_angles = client.GetSegmentGlobalRotationEulerXYZ(Drone, Drone)
                                Drone_orientation = Euler_angles[0]

                            #We get the actual position of Drone expressend in the Vicon frame:
                            D_T_tuple = client.GetSegmentGlobalTranslation( Drone, Drone )
                            D_T_millimeters = D_T_tuple[0]
                            D_T_meters = np.array([float(D_T_millimeters[0])/1000 , float(D_T_millimeters[1])/1000, float(D_T_millimeters[2])/1000])
                            D_T_vicon = D_T_meters

                            #if it is equal to [0,0,0], it meas that en error from vicon occurred so we use the last correct position instead of it.
                            if(D_T_meters[0] == 0 and D_T_meters[1] == 0 and D_T_meters[2] == 0):
                                D_T_meters = last_drone_position
                            #otherwise we convert the position in the body frame before sending it for the update of the Kalman filter:
                            else:
                                D_T_homogenous = np.array([D_T_meters[0], D_T_meters[1], D_T_meters[2], 1])
                                D_T_homogenous = np.dot(Matrix_homogeneous, np.transpose(D_T_homogenous))
                                D_T_meters = np.array([D_T_homogenous[0], D_T_homogenous[1], D_T_homogenous[2]])

                                #update the last correct value for the drone position:
                                last_drone_position = D_T_meters

                            if KALMAN_INCLUDE_QUATERNION:
                                #Update the Kalman filter sending the last measure of both the drone position and orientation:
                                cf.extpos.send_extpose(D_T_meters[0], D_T_meters[1], D_T_meters[2], quaternion[0], quaternion[1], quaternion[2], quaternion[3])
                            else:
                                #Update the Kalman filter sending the last measure of the drone position:
                                cf.extpos.send_extpos(D_T_meters[0], D_T_meters[1], D_T_meters[2])

                            if MAKE_LOG_FILE:
                                #We write in the log file with the following format:
                                #       drone's position body frame                         quaternions                                             drone's position vicon frame            drone's orientation from Vicon                                            setpoint body frame                         drone's position and orientation from log table
                                print(D_T_meters[0], D_T_meters[1], D_T_meters[2], quaternion[0], quaternion[1], quaternion[2], quaternion[3], D_T_vicon[0], D_T_vicon[1], D_T_vicon[2], Drone_orientation[0], Drone_orientation[1], Drone_orientation[2], W_T_meters[0], W_T_meters[1], W_T_meters[2], 0, log_pos_x, log_pos_y, log_pos_z, log_roll, log_pitch, log_yaw , file=fdesc)

                    #In case something wrong happens, we manage the exception with the start of the landing procedure:
                    except ViconDataStream.DataStreamException as e:
                        print( 'START LANDING: This error form Vicon occurred: \n', e )
                        landing(last_position_send,SUBTRACTED_HEIGHT,DELTA_HEIGHT,cf,lg_stab,fdesc,MAKE_LOG_FILE)

#------------------------------------------------------------------------------------------------------------------------------------------------------------
#----------------------------------UNABLED THRUSTER MODE-----------------------------------------------------------------------------------------------------
#------------------------------------------------------------------------------------------------------------------------------------------------------------

    else:
    #Here we have the condition LOG_TEST_WITH_DISACTIVATED_THRUSTER == 1
        print("START TESTING MODE WITH DISACTIVATED THRUSTER...")
        try:
            while(1):

                #Get postion of the drone in the Vicon reference System:
                a=client.GetFrame()
                b=client.GetFrameNumber()
                D_T_tuple = client.GetSegmentGlobalTranslation( Drone, Drone )
                D_T_millimeters = D_T_tuple[0]
                D_T_meters = np.array([float(D_T_millimeters[0])/1000 , float(D_T_millimeters[1])/1000, float(D_T_millimeters[2])/1000])
                #Used for store the Drone's position in the Vicon System.
                D_T_vicon = D_T_meters

                if MAKE_LOG_FILE:
                    #Get orientation of the drone from the Vicon:
                    Euler_angles = client.GetSegmentGlobalRotationEulerXYZ(Drone, Drone)
                    Drone_orientation = Euler_angles[0]

                #We convert vectors in homogenous vector and we convert the position in the body frame:
                Matrix_homogeneous, last_gamma = create_Matrix_Rotation(client, Drone, First_Position, last_gamma, KALMAN_INCLUDE_QUATERNION)
                D_T_homogenous = np.array([D_T_meters[0], D_T_meters[1], D_T_meters[2], 1])
                D_T_homogenous = np.dot(Matrix_homogeneous, np.transpose(D_T_homogenous))
                D_T_meters = np.array([D_T_homogenous[0], D_T_homogenous[1], D_T_homogenous[2]])

                #if KALMAN_INCLUDE_QUATERNION:
                #We decide to get the quaternion even in the case KALMAN_INCLUDE_QUATERNION=0 because in this way we maintain the same "computational cost" for both the case =0 and =1
                quaternion_XYZW = client.GetSegmentGlobalRotationQuaternion(Drone, Drone)
                quaternion = quaternion_XYZW[0]

                if MAKE_LOG_FILE:
                    #We write in the log file with the following format:
                    #       drone's position body frame                         quaternions                                                 drone's position vicon frame                drone's orientation from Vicon                              setpoint body frame             drone's position and orientation from log table
                    print(D_T_meters[0], D_T_meters[1], D_T_meters[2], quaternion[0], quaternion[1], quaternion[2], quaternion[3], D_T_vicon[0], D_T_vicon[1], D_T_vicon[2], Drone_orientation[0], Drone_orientation[1], Drone_orientation[2], 0, 0, DEFAULT_HEIGHT, 0, log_pos_x, log_pos_y, log_pos_z, log_roll, log_pitch, log_yaw , file=fdesc)

                if ACTIVATE_KALMAN_DURING_TAKEOFF:
                #Send measures to Kalman filter also during the take off.
                #In this case there isn't a proper "take off phase" so with this flag = 1 we obtain the update of the kalman filter during the enitre experiment.
                #and with the value =0 we don't send information to Kalman during the entire experiment.
                    if KALMAN_INCLUDE_QUATERNION:
                        cf.extpos.send_extpose(D_T_meters[0], D_T_meters[1], D_T_meters[2], quaternion[0], quaternion[1], quaternion[2], quaternion[3])
                    else:
                        cf.extpos.send_extpos(D_T_meters[0], D_T_meters[1], D_T_meters[2])

        #In case something wrong happens, we manage the exception with the start of the landing procedure:
        except ViconDataStream.DataStreamException as e:
            print( 'STOP TEST: This error form Vicon occurred: \n', e )

            if MAKE_LOG_FILE:
                #close the connection with the log table:
                lg_stab.stop()
                fdesc.close()

            exit()
#-------------------------------------------------------------------------------------------------------------------------------------------------------------
#---------------------------------------END FILE--------------------------------------------------------------------------------------------------------------
#-------------------------------------------------------------------------------------------------------------------------------------------------------------
