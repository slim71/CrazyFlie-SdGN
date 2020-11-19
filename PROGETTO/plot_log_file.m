%Change "filepath" with the correct path of the file you want to open
filepath = "/LOG/Log_File_CrazyFlie_20201027_092159_ThrusterOn__KalmanDuringTakeOff_KalmanWithoutQuaternion";

DATA = importdata(filepath);

%Extract data:                          Variables meanings: 
pos_sent_toKF = DATA(:,1:3);        %   D_T_meters[0:2]     --> posizione drone presa da vicon, ruotata e inviata al drone per l'aggiornamento del filtro di Kalman 
quat_sent_toKF = DATA(:,4:7);       %   quaternion[0:2]     --> quaternioni per l'orientazione del drone presi da vicon e inviati (o non inviati) al drone per l'aggiornamento del filtro di Kalman
vicon_xyz = DATA(:,8:10);           %   D_T_vicon[0:2]      --> Posizione drone presa da vicon, non ruotata
vicon_rpy = DATA(:,11:13);          %   angles[0:2]         --> roll pitch yaw del drone, presi da vicon 
setpoint = DATA(:,14:17);           %   W_T_meters[0:2],0   --> setpoint inviato al drone nel formato "x,y,z,yaw" (nel caso in cui gli si voglia far inseguire il moto della bacchetta)
log_xyz = DATA(:,18:20);            %   log_pos_x, log_pos_y, log_pos_z --> posizione del drone presa dalla tabella di log del crazyflie
log_rpy = DATA(:,21:23).*pi/180.0;  %   log_roll, log_pitch, log_yaw    --> orientazione del drone presa dalla tabella di log del crazyflie



%Confronto tra angoli RPY Vicon && angoli RPY Log Table 
figure,
subplot(3,1,1), plot(vicon_rpy(:,1),'r'); hold on; plot(log_rpy(:,1),'b'); hold off; grid on; legend('vicon','log');
subplot(3,1,2), plot(vicon_rpy(:,2),'r'); hold on; plot(log_rpy(:,2),'b'); hold off; grid on; 
subplot(3,1,3), plot(vicon_rpy(:,3),'r'); hold on; plot(log_rpy(:,3),'b'); hold off; grid on; 


%Confronto tra posizione XYZ Log Table && posizione XYZ Vicon && posizione XYZ inviata al filtro di Kalman (quella del Vicon ma roto-traslata)

figure
subplot(3,1,1), plot(log_xyz(:,1), 'b'); hold on; plot(vicon_xyz(:,1), 'r'); hold on; plot(pos_sent_toKF(:,1), 'g'); hold off; grid on; legend('log','vicon','kalman');
subplot(3,1,2), plot(log_xyz(:,2), 'b'); hold on; plot(vicon_xyz(:,2), 'r'); hold on; plot(pos_sent_toKF(:,2), 'g'); hold off; grid on;
subplot(3,1,3), plot(log_xyz(:,3), 'b'); hold on; plot(vicon_xyz(:,3), 'r'); hold on; plot(pos_sent_toKF(:,3), 'g'); hold off; grid on;

