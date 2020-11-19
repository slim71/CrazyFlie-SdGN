#------------------------------------COMMENTS----------------------------------------------------------------------------------------------------------------
#------------------------------------------------------------------------------------------------------------------------------------------------------------
#------------------------------------------------------------------------------------------------------------------------------------------------------------
"""
NB:
    TEST
    1) Esecuzione di questo file per inseguire il moto della wand ogni WAIT_TIME secondi
    2) Eliminazione del vincolo di attesa di WAIT_TIME e inseguimento del moto della wand in tempo reale con un offset di sicurezza
    3) Provare spengimento wand e fine esperimento
    4) Aggiungere la partenza da un qualsiasi punto della stanza e le relative conversioni tra sistemi di riferimento
    -->5) Aggiungere kalman con spostamento relativo invece che inseguimento

"""
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
from cflib.crtp.crtpstack import CRTPPacket
from cflib.crtp.crtpstack import CRTPPort
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

T_prec = np.array([0, 0, 0])                #Used in the generation of the position reference for the drone
W_T_prec = np.array([0, 0, 0])              #Used in the generation of the position reference for the drone
take_off = 1                                #It will be set to 0 after the take off
last_gamma = 0

WAIT_TIME = 5 #[s]                          #Used for the management of the iterations
SLEEP_TIME = 0.001 #[s]                     #Used for the management of the iterations
ITERATIONS = WAIT_TIME/SLEEP_TIME           #Used for the management of the iterations
OFFSET=0.3 #[m]                             #Security offset
MAX_LOSS = 5                                #Max number of loss during acquisition of the position of the Wand or Drone
CONSECUTIVE_LOSS = 0                        #Current number of consecutive loss in the acquisition of the wand position
DELTA_HEIGHT=0.01 #[m]                      #[m] to be subtracted from the "z" component of the last position of the drone during each iteration of the landing phase
SUBTRACTED_HEIGHT = 0.01 #[m]               #Sum of [m] subtracted from the  "z" component of the last position of the drone
DEFAULT_HEIGHT = 0.5 #[m]                   #The height that the drone reachs after take off 
take_off=0
i=0

#------------------------------------------------------------------------------------------------------------------------------------------------------------
#-------------------------------FUNCTIONS--------------------------------------------------------------------------------------------------------------------
#------------------------------------------------------------------------------------------------------------------------------------------------------------

def get_First_Position(client, Drone):
    #We get the start position of the Drone and we convert it in an homogeneus position vector
    D_T_meters=np.array([0, 0, 0])
    iterations= 0
    #we have to be sure we took the right position
    while(D_T_meters[0] == 0 and D_T_meters[1] == 0 and D_T_meters[2] == 0):
        a=client.GetFrame()                                                                   #It is necessary before every time we want to call some "GetSegment..()"
        b=client.GetFrameNumber()
        D_T_tuple = client.GetSegmentGlobalTranslation( Drone, Drone )                        #Absolute position of the Drone in [mm]
        D_T_millimeters= D_T_tuple[0]                                                         #We are interested only in the first part of the data structure
        D_T_meters_homogeneous = np.array([float(D_T_millimeters[0])/1000 , float(D_T_millimeters[1])/1000, float(D_T_millimeters[2])/1000, 1])    #Pass from [mm] to [m]
        D_T_meters= np.array([D_T_meters_homogeneous[0], D_T_meters_homogeneous[1], D_T_meters_homogeneous[2]])
        #If for 10 times we get the value (0, 0, 0), we can consider that is the right one
        if(iterations > MAX_LOSS):
            print("WARNING: initial position of the Drone is exactly [0,0,0] for ", MAX_LOSS, " consecutive times. Please try with another initial position and restart the experiment.")
            exit()
        else:
            iterations = iterations + 1

    return D_T_meters

def create_Matrix_Rotation(client, Drone, D_T_meters, last_gamma):
    #We get the start orientation of the Drone from which we want to get the RPY angles
    Euler_XYZ= client.GetSegmentGlobalRotationEulerXYZ(Drone, Drone)
    Euler_radians = Euler_XYZ[0]                #We are interested only in the first part of the tuple
    print("EULER Angles:", Euler_radians)
    if(Euler_radians[0] == 0 and Euler_radians[1] == 0 and Euler_radians[2] == 0 ):
        gamma = last_gamma
    else:
        gamma = Euler_radians[2]
        last_gamma = gamma

    alpha = 0                                  #We are interested only at the yaw angle
    beta = 0
    #From angles we build the homogeneous rotation matrix that will convert the positions expressed in the Vicon reference system in positions expressed in the body frame of the Drone
    R_x = np.array([[1 , 0, 0], [0, math.cos(alpha), -math.sin(alpha)], [0, math.sin(alpha), math.cos(alpha)]])
    R_y = np.array([[math.cos(beta), 0, math.sin(beta)], [0, 1, 0], [-math.sin(beta), 0, math.cos(beta)]])
    R_z = np.array([[math.cos(gamma), -math.sin(gamma), 0], [math.sin(gamma), math.cos(gamma), 0], [0, 0, 1]])
    Matrix_Rotation_xy = np.dot(R_x, R_y)
    Matrix_Rotation_xyz = np.dot(Matrix_Rotation_xy, R_z)
    Matrix_Rotation = np.transpose(Matrix_Rotation_xyz)
    Matrix_homogeneous= np.array([[Matrix_Rotation[0][0], Matrix_Rotation[0][1], Matrix_Rotation[0][2], -np.dot(Matrix_Rotation, np.transpose(D_T_meters))[0]], [Matrix_Rotation[1][0], Matrix_Rotation[1][1], Matrix_Rotation[1][2], -np.dot(Matrix_Rotation, np.transpose(D_T_meters))[1]], [Matrix_Rotation[2][0], Matrix_Rotation[2][1], Matrix_Rotation[2][2], -np.dot(Matrix_Rotation, np.transpose(D_T_meters))[2]], [0, 0, 0, 1]])
    #print("Matrix Homogenous:", Matrix_homogeneous)

    return Matrix_homogeneous, Matrix_Rotation, last_gamma

#------------------------------------------------------------------------------------------------------------------------------------------------------------
#----------------------------------VICON CONNECTION----------------------------------------------------------------------------------------------------------
#------------------------------------------------------------------------------------------------------------------------------------------------------------
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--hostname", "--"+VICON_IP+":"+VICON_PORT)
args = parser.parse_args()

#Create a VICON Client
client = ViconDataStream.Client()
i=0
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
#----------------------------------MAIN----------------------------------------------------------------------------------------------------------------------
#------------------------------------------------------------------------------------------------------------------------------------------------------------

#DRONE CONNECTION
logging.basicConfig(level=logging.ERROR) 
cflib.crtp.init_drivers(enable_debug_driver=False)
cf=cflib.crazyflie.Crazyflie()
First_Position = get_First_Position(client, Drone)
with SyncCrazyflie(uri, cf) as scf:
    with MotionCommander(scf, default_height = 0.55) as mc:
        print('TAKE OFF!')
        height_drone = 0                                                                              
        while(height_drone < 0.5):
            a=client.GetFrame()
            b=client.GetFrameNumber()
            D_T_tuple = client.GetSegmentGlobalTranslation( Drone, Drone )
            D_T_millimeters = D_T_tuple[0]
            D_T_meters = np.array([float(D_T_millimeters[0])/1000 , float(D_T_millimeters[1])/1000, float(D_T_millimeters[2])/1000])
            height_drone = D_T_meters[2] 

        last_drone_position = np.array([0, 0, height_drone])
        last_drone_reference = np.array([0, 0, height_drone])
        print(last_drone_reference)
        sdf=0
        while(1):

            if(take_off==0):                                                                       #take off already done
                try:
                    if(i==1):
                        print("POSIZIONE BACCHETTA: ", W_T_meters)
                    a=client.GetFrame()
                    b=client.GetFrameNumber()                                                      #It is necessary before every time we want to call some "GetSegment..()"
                    Matrix_homogeneous, Matrix_Rotation, last_gamma = create_Matrix_Rotation(client, Drone, First_Position, last_gamma)
                    W_T_tuple = client.GetSegmentGlobalTranslation( Wand, 'Root' )                 #Absolute position of the Drone in [mm]
                    W_T_millimeters= W_T_tuple[0]                                                  #We are interested only in the first part of the data structure
                    W_T_meters = np.array([float(W_T_millimeters[0])/1000 , float(W_T_millimeters[1])/1000, float(W_T_millimeters[2])/1000])    #Pass from [mm] to [m]
                    #print("NUOVA POSIZIONE BACCHETTA: ", W_T_meters)
                    #print("GET FRAME: ", W_T_meters[0], W_T_meters[1], W_T_meters[2])


                    if(W_T_meters[0]==0 and W_T_meters[1]==0 and W_T_meters[2]==0):
                        CONSECUTIVE_LOSS=CONSECUTIVE_LOSS+1
                        cf.commander.send_position_setpoint(last_drone_reference[0], last_drone_reference[1], last_drone_reference[2], 0)
                        if(CONSECUTIVE_LOSS==MAX_LOSS):
                            #LANDING PHASE: We received MAX_LOSS consecutive "null position" of the wand so we decide to start landing.
                            print("START LANDING: ", MAX_LOSS, " consecutive null position of the Wand have been received.")
                            while(last_drone_reference[2]-SUBTRACTED_HEIGHT>0):
                                cf.commander.send_position_setpoint(last_drone_reference[0], last_drone_reference[1], last_drone_reference[2]-SUBTRACTED_HEIGHT, 0)
                                SUBTRACTED_HEIGHT=SUBTRACTED_HEIGHT+DELTA_HEIGHT
                            exit()
                    else:
                        #We pass from a Wand's position expressed in the Vicon frame to a Wand's position expressed in the Body frame of the Drone before sending it
                        if(i==0 or CONSECUTIVE_LOSS):

                            Wand_Traslation = np.array([0, 0, 0])
                            W_T_prec = np.array([W_T_meters[0], W_T_meters[1], W_T_meters[2]])
                            CONSECUTIVE_LOSS = 0 
                            
                        else:
                            CONSECUTIVE_LOSS = 0
                            Wand_Traslation[0] = W_T_meters[0] - W_T_prec[0]
                            Wand_Traslation[1] = W_T_meters[1] - W_T_prec[1]
                            Wand_Traslation[2] = W_T_meters[2] - W_T_prec[2]
                            W_T_prec = np.array([W_T_meters[0], W_T_meters[1], W_T_meters[2]])

                        if(i==1):    
                            Wand_Traslation = np.dot(Matrix_Rotation, np.transpose(Wand_Traslation))
                            print("TRASLAZIONE BODY: ", Wand_Traslation)
                        #print("RIFERIMENTO DRONE: ", last_drone_reference)
                        last_drone_reference[0] = last_drone_reference[0] + Wand_Traslation[0]
                        last_drone_reference[1] = last_drone_reference[1] + Wand_Traslation[1]
                        last_drone_reference[2] = last_drone_reference[2] + Wand_Traslation[2]
                        print("NUOVO RIFERIMENTO DRONE: ", last_drone_reference, "\n\n")
                        i=1
                        #We send the actual position of Drone expressend in the Body frame to the Kalman Filter
                        D_T_tuple = client.GetSegmentGlobalTranslation( Drone, Drone )
                        D_T_millimeters = D_T_tuple[0]
                        D_T_meters = np.array([float(D_T_millimeters[0])/1000 , float(D_T_millimeters[1])/1000, float(D_T_millimeters[2])/1000])
                        if(D_T_meters[0] == 0 and D_T_meters[1] == 0 and D_T_meters[2] == 0):
                            D_T_meters = last_drone_position
                        else:
                            D_T_homogenous = np.array([D_T_meters[0], D_T_meters[1], D_T_meters[2], 1])
                            D_T_homogenous = np.dot(Matrix_homogeneous, np.transpose(D_T_homogenous))
                            D_T_meters = np.array([D_T_homogenous[0], D_T_homogenous[1], D_T_homogenous[2]])
                            last_drone_position = D_T_meters
                            
                        cf.extpos.send_extpos(D_T_meters[0], D_T_meters[1], D_T_meters[2])
                        print("DOVE SI TROVA IL DRONE: ", D_T_meters[0], D_T_meters[1], D_T_meters[2])
                        #We send the new setpoint
                        cf.commander.send_position_setpoint(last_drone_reference[0], last_drone_reference[1], last_drone_reference[2], 0)
                        sdf = sdf + 0.05
                        #print("POSITION SEND: ", last_drone_reference[0], last_drone_reference[1], last_drone_reference[2])                      

                except ViconDataStream.DataStreamException as e:
                    print( 'START LANDING: This error form Vicon occurred: \n', e )
                    #LANDING PHASE: We received MAX_LOSS consecutive "null position" of the wand so we decide to start landing.
                    while(last_drone_reference[2]-SUBTRACTED_HEIGHT>0):
                        cf.commander.send_position_setpoint(last_drone_reference[0], last_drone_reference[1], last_drone_reference[2]-SUBTRACTED_HEIGHT, 0)
                        SUBTRACTED_HEIGHT=SUBTRACTED_HEIGHT+DELTA_HEIGHT
                    exit()

#-------------------------------------------------------------------------------------------------------------------------------------------------------------
#-------------------------------------------------------------------------------------------------------------------------------------------------------------
#-------------------------------------------------------------------------------------------------------------------------------------------------------------
