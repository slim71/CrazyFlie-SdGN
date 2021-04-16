%Change "filepath" with the correct path of the file you want to open
% filepath = "pitchOnly.txt";
filepath = "__20210330_1352.txt";

DATA = importdata(filepath);

%Extract data:                          Variables meanings: 
pos_sent_toKF = DATA.data(1:2:end,1:3);        %   D_T_meters[0:2]     --> posizione drone presa da vicon, ruotata e inviata al drone per l'aggiornamento del filtro di Kalman 
quat_sent_toKF = DATA.data(1:2:end,4:7);       %   quaternion[0:2]     --> quaternioni per l'orientazione del drone presi da vicon e inviati (o non inviati) al drone per l'aggiornamento del filtro di Kalman
quat_sent_toKF = [quat_sent_toKF(:,4),quat_sent_toKF(:,1),quat_sent_toKF(:,2),quat_sent_toKF(:,3)]; % change orger from CF to MATLAB
[yaw, pitch, roll] = quat2angle(quat_sent_toKF);
yaw = rad2deg(yaw); pitch = -rad2deg(pitch); roll = rad2deg(roll);

log_xyz = DATA.data(2:2:end,1:3);            %   log_pos_x, log_pos_y, log_pos_z --> posizione del drone presa dalla tabella di log del crazyflie
log_ypr = DATA.data(2:2:end,4:6);            %   log_yaw, log_pitch, log_roll    --> orientazione del drone presa dalla tabella di log del crazyflie


figure
subplot(3,1,1), plot(roll,'r'), title("YAW"); hold on; plot(log_ypr(:,1), 'b'), legend("vicon", "estim");
subplot(3,1,2), plot(pitch,'r'), title("PITCH"); hold on; plot(log_ypr(:,2), 'b');
subplot(3,1,3), plot(yaw,'r'), title("ROLL"); hold on; plot(log_ypr(:,3), 'b');

figure
subplot(3,1,1), plot(pos_sent_toKF(:,1), 'r'), title("X"); hold on; plot(log_xyz(:,1), 'b'), legend("vicon", "estim");
subplot(3,1,2), plot(pos_sent_toKF(:,2), 'r'), title("Y"); hold on; plot(log_xyz(:,2), 'b'), legend("vicon", "estim");
subplot(3,1,3), plot(pos_sent_toKF(:,3), 'r'), title("Z"); hold on; plot(log_xyz(:,3), 'b'), legend("vicon", "estim");
