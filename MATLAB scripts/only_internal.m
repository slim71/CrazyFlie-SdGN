%% File loading
internal = "C:\Users\Vicon-PC\Desktop\PROGETTO CRAZYFLIE\CrazyFlie-SdGN\data_logs\sequence_flight__20210330_1120.txt";

internal_data = importdata(internal);

%% Data extraction
% Extracted data                        Variables meaning 

int_px = internal_data(:,1);            % \
int_py = internal_data(:,2);            %  |-> internal estimate of drone position
int_pz = internal_data(:,3);            % /
int_roll = internal_data(:,4);          % \
int_pitch = internal_data(:,5);         % |-> internal estimate of drone attitude
int_yaw = internal_data(:,6);           % /

%% Analysis
% MATLAB uses q = [w x y z]

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
plot(crazy_euler(:,1),'b')
legend('Vicon', 'Estimate')
title("Roll")

subplot(3,1,2)
hold on
grid on
plot(crazy_euler(:,2),'b')
legend('Vicon', 'Estimate')
title("Pitch")

subplot(3,1,3)
hold on
grid on
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
plot(int_px,'b')
legend('Vicon', 'Estimate')
title("X coordinates")

subplot(3,1,2)
hold on
grid on
plot(int_py,'b')
legend('Vicon', 'Estimate')
title("Y coordinates")

subplot(3,1,3)
hold on
grid on
plot(int_pz,'b')
legend('Vicon', 'Estimate')
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

plot3(int_px, int_py, int_pz, '-b')
title("Crazyflie reference system")