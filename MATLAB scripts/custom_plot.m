%% File loading
name = "sequence_flight__20210415_1745.txt";
current_file = mfilename('fullpath');
[path, ~, ~] = fileparts(current_file);

internal = fullfile(path, '..\internal_data\', name);
setpoints = fullfile(path, '..\setpoint_data\', name);
vicon = fullfile(path, '..\vicon_data\', name); % TODO: redundant?
wand = fullfile(path, '..\wand_data\', name);

raw_internal_data = importdata(internal);
raw_set_data = importdata(internal);
raw_vicon_data = importdata(vicon); % TODO: redundant?
raw_wand = impordata(wand);

if isstruct(raw_internal_data)
    internal_data = raw_internal_data.data;
else
    internal_data = raw_internal_data;
end

if isstruct(raw_set_data)
    set_data = raw_set_data.data;
else
    set_data = raw_set_data;
end

if isstruct(raw_vicon_data)
    vicon_data = raw_vicon_data.data;
else
    vicon_data = raw_vicon_data;
end

if isstruct(raw_wand)
    wand_data = raw_wand.data;
else
    wand_data = raw_wand;
end

%% Data extraction
% Extracted data                        Variables meaning 
drone_posx = vicon_data(:,1);           % \
drone_posy = vicon_data(:,2);           %  |-> drone position from Vicon, in Vicon frame [m]
drone_posz = vicon_data(:,3);           % /
drone_quatx = vicon_data(:,4);          %  \
drone_quaty = vicon_data(:,5);          %   |-> drone orientation through quaternions from Vicon 
drone_quatz = vicon_data(:,6);          %  /
drone_quatw = vicon_data(:,7);          % /
cust_time = datetime(vicon_data(:,end), 'ConvertFrom', 'datenum');

setx_v = set_data(:,1);                 % \
sety_v = set_data(:,2);                 %  |-> setpoint coordinates in Vicon reference system
setz_v = set_data(:,3);                 % /
setx_cf = set_data(:,4);                % \
sety_cf = set_data(:,5);                %  |-> setpoint coordinates in Crazyflie reference system
setz_cf = set_data(:,6);                % /
set_time = datetime(set_data(:,end), 'ConvertFrom', 'datenum');

int_px = internal_data(:,1);            % \
int_py = internal_data(:,2);            %  |-> internal estimate of drone position
int_pz = internal_data(:,3);            % /
int_roll = internal_data(:,4);          % \
int_pitch = internal_data(:,5);         % |-> internal estimate of drone attitude
int_yaw = internal_data(:,6);           % /
int_time = datetime(internal_data(:,end), 'ConvertFrom', 'datenum');

wand_px = wand_data(:,1);               % \
wand_py = wand_data(:,2);               %  |-> Wand position in Vicon frame
wand_pz = wand_data(:,3);               % /
wand_time = datetime(wand_data(:,end), 'ConvertFrom', 'datenum');

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
    figure('name', "Comparison between Euler angles")
else
    figure2('name', "Comparison between Euler angles")
end

subplot(3,1,1)
hold on
grid on
plot(cust_time, vicon_euler(:,1),'r')
plot(int_time, crazy_euler(:,1),'b')
ylabel("degree [°]")
legend('Vicon', 'Estimate')
title("Roll")

subplot(3,1,2)
hold on
grid on
plot(cust_time, vicon_euler(:,2),'r')
plot(int_time, crazy_euler(:,2),'b')
ylabel("degree [°]")
legend('Vicon', 'Estimate')
title("Pitch")

subplot(3,1,3)
hold on
grid on
plot(cust_time, vicon_euler(:,3),'r')
plot(int_time, crazy_euler(:,3),'b')
ylabel("degree [°]")
legend('Vicon', 'Estimate')
title("Yaw")

%% Comparison between positions
% This section analyzes the drone's internal position estimation with the
% Vicon-captured position

if exist('figure2') == 0  %#ok<*EXIST>
    figure('name', "Comparison between positions")
else
    figure2('name', "Comparison between positions")
end

subplot(3,1,1)
hold on
grid on
plot(cust_time, drone_posx,'r')
plot(int_time, int_px,'b')
ylabel("meter [m]")
legend('Vicon', 'Estimate')
title("X coordinates")

subplot(3,1,2)
hold on
grid on
plot(cust_time, drone_posy,'r')
plot(int_time, int_py,'b')
ylabel("meter [m]")
legend('Vicon', 'Estimate')
title("Y coordinates")

subplot(3,1,3)
hold on
grid on
plot(cust_time, drone_posz,'r')
plot(int_time, int_pz,'b')
ylabel("meter [m]")
legend('Vicon', 'Estimate')
title("Z coordinates")

%% Setpoint visualization in Vicon reference system

if exist('figure2') == 0  %#ok<*EXIST>
    figure('name', "Setpoint visualization in Vicon reference system")
else
    figure2('name', "Setpoint visualization in Vicon reference system")
end

subplot(3,1,1)
hold on
grid on
plot(cust_time, setx_v, 'o-')
title("X coordinates")

subplot(3,1,2)
hold on
grid on
plot(cust_time, setx_v, 'o-')
title("Y coordinates")

subplot(3,1,3)
hold on
grid on
plot(cust_time, setz_v, 'o-')
title("Z coordinates")

%% Setpoint visualization in Crazyflie reference system

if exist('figure2') == 0  %#ok<*EXIST>
    figure('name', "Setpoint visualization in Crazyflie reference system")
else
    figure2('name', "Setpoint visualization in Crazyflie reference system")
end

subplot(3,1,1)
hold on
grid on
plot(int_time, setx_cf, 'o')
title("X coordinates")

subplot(3,1,2)
hold on
grid on
plot(int_time, setx_cf, 'o')
title("Y coordinates")

subplot(3,1,3)
hold on
grid on
plot(int_time, setz_cf, 'o')
title("Z coordinates")

%% 3D visualization in Vicon reference system

if exist('figure2') == 0  %#ok<*EXIST>
    figure('name', "3D visualization in Vicon reference system")
else
    figure2('name', "3D visualization in Vicon reference system")
end

hold on
grid on
view(10,45)
xlabel("Vicon x axis [m]")
ylabel("Vicon y axis [m]")
zlabel("Vicon z axis [m]")

plot3(setx_v, sety_v, setz_v, 'o', 'Color', 'b', ...
    'MarkerSize', 10, 'MarkerFaceColor', '#D9FFFF')
plot3(drone_posx, drone_posy, drone_posz, '-b')
plot_order(setx_v, sety_v, setz_v)

legend("Setpoint", "Drone position")
title("Vicon reference system")

%% 3D visualization in Vicon reference system

if exist('figure2') == 0  %#ok<*EXIST>
    figure('name', "3D visualization in Vicon reference system")
else
    figure2('name', "3D visualization in Vicon reference system")
end

hold on
grid on
view(10,45)
xlabel("x")
ylabel("y")
zlabel("z")

plot3(setx_cf, sety_cf, setz_cf, 'o', 'Color', 'b', ...
    'MarkerSize', 10, 'MarkerFaceColor', '#D9FFFF')
plot3(int_px, int_py, int_pz, '--k')
plot_order(setx_cf, sety_cf, setz_cf)
extremities(int_px, int_py, int_pz)
legend("Setpoint", "Drone position")
title("Crazyflie reference system")

%% Wand position visualization, with drone Vicon position

if exist('figure2') == 0  %#ok<*EXIST>
    figure('name', "Wand and drone position visualization, in Vicon frame");
else
    figure2('name', "Wand and drone position visualization, in Vicon frame");
end

hold on
grid on
view(10,45)
xlabel("x")
ylabel("y")
zlabel("z")

plot3(wand_px, wand_py, wand_pz, '--k')
plot3(setx_cf, sety_cf, setz_cf, 'o', 'Color', 'b', ...
    'MarkerSize', 10, 'MarkerFaceColor', '#D9FFFF')
plot_order(setx_cf, sety_cf, setz_cf)
extremities(int_px, int_py, int_pz)
legend("Wand position", "Drone position")
title("Vicon reference system")