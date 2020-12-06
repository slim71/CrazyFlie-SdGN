%% File loading
% Change "filepath" with the correct path of the file you want to open
filepath = ...
    "C:\Users\User\Documents\GitHub\CrazyFlie-SdGN\PROGETTO\LOG\Log_File_CrazyFlie_20201015_105731_ThrusterOff__KalmanDisabled.txt";

DATA = importdata(filepath);

%% Data extraction
% Extracted data                        Variables meaning 
pos_sent_toKF = DATA(:,1:3);        %   D_T_meters[0:2]     --> posizione drone presa da vicon, ruotata e inviata al drone per l'aggiornamento del filtro di Kalman 
quat_sent_toKF = DATA(:,4:7);       %   quaternion[0:2]     --> quaternioni per l'orientazione del drone presi da vicon e inviati (o non inviati) al drone per l'aggiornamento del filtro di Kalman
vicon_xyz = DATA(:,8:10);           %   D_T_vicon[0:2]      --> Posizione drone presa da vicon, non ruotata
vicon_rpy = DATA(:,11:13);          %   angles[0:2]         --> roll pitch yaw del drone, presi da vicon 
setpoint = DATA(:,14:17);           %   W_T_meters[0:2],0   --> setpoint inviato al drone nel formato "x,y,z,yaw" (nel caso in cui gli si voglia far inseguire il moto della bacchetta)
log_xyz = DATA(:,18:20);            %   log_pos_x, log_pos_y, log_pos_z --> posizione del drone presa dalla tabella di log del crazyflie
log_rpy = DATA(:,21:23).*pi/180.0;  %   log_roll, log_pitch, log_yaw    --> orientazione del drone presa dalla tabella di log del crazyflie

%% Analysis
% Comparison between Vicon and LogTable RPY angles
if exist('figure2') == 0  %#ok<*EXIST>
    figure()
else
    figure2()
end

sgtitle("RPY angle comparison")
% xlabel()
% ylabel()

subplot(3,1,1)
hold on
grid on
plot(vicon_rpy(:,1),'r')
plot(log_rpy(:,1),'b')
legend('Vicon','LogTable')

subplot(3,1,2)
hold on
grid on
plot(vicon_rpy(:,2),'r')
plot(log_rpy(:,2),'b')

subplot(3,1,3)
hold on
grid on
plot(vicon_rpy(:,3),'r')
plot(log_rpy(:,3),'b')

% Comparison between Vicon, LogTable and sent-to-KF positions (XYZ)
% (The last one is the Vicon one, but rototranslated)
if exist('figure2') == 0
    figure()
else
    figure2()
end

sgtitle("Positions comparison")
% xlabel()
% ylabel()

subplot(3,1,1)
hold on
grid on
plot(log_xyz(:,1), 'b')
plot(vicon_xyz(:,1), 'r')
plot(pos_sent_toKF(:,1), 'g')
legend('LogTable','Vicon','Kalman')

subplot(3,1,2)
hold on
grid on
plot(log_xyz(:,2), 'b')
plot(vicon_xyz(:,2), 'r')
plot(pos_sent_toKF(:,2), 'g')

subplot(3,1,3)
hold on
grid on
plot(log_xyz(:,3), 'b')
plot(vicon_xyz(:,3), 'r')
plot(pos_sent_toKF(:,3), 'g')
