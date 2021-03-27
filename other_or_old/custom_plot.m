%% File loading
internal = "C:\Users\User\Documents\GitHub\CrazyFlie-SdGN\data_logs\__20210323_1526.txt";
custom = "C:\Users\User\Documents\GitHub\CrazyFlie-SdGN\matlab_logs\__20210323_1526.txt";

internal_data = importdata(internal);
custom_data = importdata(custom);

%% Data extraction
% Extracted data                        Variables meaning 
drone_posx = custom_data(:,1);          % \
drone_posy = custom_data(:,2);          %  |-> drone position from Vicon, in Vicon frame
drone_posz = custom_data(:,3);          % /
is_pos_blocked = custom_data(:,4);      % flag passed from the Vicon tracker
drone_quatx = custom_data(:,5);         %  \
drone_quaty = custom_data(:,6);         %   |-> drone orientation through quaternions from Vicon 
drone_quatz = custom_data(:,7);         %  /
drone_quatw = custom_data(:,8);         % /
is_or_blocked = custom_data(:,9);       % flag passed from the Vicon tracker
setx_v = custom_data(:,10);             % \
sety_v = custom_data(:,11);             %  |-> setpoint coordinates in Vicon reference system
setz_v = custom_data(:,12);             % /
setx_cf = custom_data(:,13);            % \
sety_cf = custom_data(:,14);            %  |-> setpoint coordinates in Crazyflie reference system
setz_cf = custom_data(:,15);            % /

int_px = internal_data(:,1);            % \
int_py = internal_data(:,2);            %  |-> internal estimate of drone position
int_pz = internal_data(:,3);            % /
int_roll = internal_data(:,4);          % \
int_pitch = internal_data(:,5);         % |-> internal estimate of drone attitude
int_yaw = internal_data(:,6);           % /

%% Analysis
% MATLAB uses q = [w x y z]
% Vicon creates q = [x y z w]

% Conversion to Euler angles from the Vicon-generated quaternions
vicon_quat = [drone_quatw, drone_quatx, drone_quaty, drone_quatz];
vicon_euler = quat2eul(vicon_quat, 'ZYX');

% orientation quaternion derived from internal estimate
crazy_euler = [int_roll, int_pitch, int_yaw];
crazy_quat = eul2quat(crazy_euler, 'ZYX');

%% Comparison between Euler angles
% This section analyzes the drone's internal attitude estimation with the
% Vicon-captured angles

if exist('figure2') == 0  %#ok<*EXIST>
    figure()
else
    figure2()
end

subplot(3,1,1)
hold on
grid on
plot(vicon_euler(:,1),'r')
plot(crazy_euler(:,1),'b')
legend('Vicon', 'Estimate')
title("Roll")

subplot(3,1,2)
hold on
grid on
plot(vicon_euler(:,2),'r')
plot(crazy_euler(:,2),'b')
legend('Vicon', 'Estimate')
title("Pitch")

subplot(3,1,3)
hold on
grid on
plot(vicon_euler(:,3),'r')
plot(crazy_euler(:,3),'b')
legend('Vicon', 'Estimate')
title("Yaw")

%% Comparison between positions
% This section analyzes the drone's internal position estimation with the
% Vicon-captured position

if exist('figure2') == 0  %#ok<*EXIST>
    figure()
else
    figure2()
end

subplot(3,1,1)
hold on
grid on
plot(drone_posx,'r')
plot(int_px,'b')
legend('Vicon', 'Estimate')
title("X coordinates")

subplot(3,1,2)
hold on
grid on
plot(drone_posy,'r')
plot(int_py,'b')
legend('Vicon', 'Estimate')
title("Y coordinates")

subplot(3,1,3)
hold on
grid on
plot(drone_posz,'r')
plot(int_pz,'b')
legend('Vicon', 'Estimate')
title("Z coordinates")

%% Setpoint visualization in Vicon reference system

if exist('figure2') == 0  %#ok<*EXIST>
    figure()
else
    figure2()
end

subplot(3,1,1)
hold on
grid on
plot(setx_v)
title("X coordinates")

subplot(3,1,2)
hold on
grid on
plot(setx_v)
title("Y coordinates")

subplot(3,1,3)
hold on
grid on
plot(setz_v)
title("Z coordinates")

%% Setpoint visualization in Crazyflie reference system

if exist('figure2') == 0  %#ok<*EXIST>
    figure()
else
    figure2()
end

subplot(3,1,1)
hold on
grid on
plot(setx_cf)
title("X coordinates")

subplot(3,1,2)
hold on
grid on
plot(setx_cf)
title("Y coordinates")

subplot(3,1,3)
hold on
grid on
plot(setz_cf)
title("Z coordinates")

%% 3D visualization

if exist('figure2') == 0  %#ok<*EXIST>
    figure()
else
    figure2()
end

hold on
grid on
view(10,45)
xlabel("x")
ylabel("y")
zlabel("z")

plot3(setx_v(:), sety_v(:), setz_v(:), 'o', 'Color', 'b', ...
    'MarkerSize', 10, 'MarkerFaceColor', '#D9FFFF')
plot3(drone_posx, drone_posy, drone_posz, '-b')
legend("Setpoint", "Drone position")
title("Vicon reference system")

if exist('figure2') == 0  %#ok<*EXIST>
    figure()
else
    figure2()
end

hold on
grid on
view(10,45)
xlabel("x")
ylabel("y")
zlabel("z")

plot3(setx_cf(:), sety_cf(:), setz_cf(:), 'o', 'Color', 'b', ...
    'MarkerSize', 10, 'MarkerFaceColor', '#D9FFFF')
plot3(int_px, int_py, int_pz, '-b')
legend("Setpoint", "Drone position")
title("Crazyflie reference system")