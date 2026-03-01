% Created by Tianyi Han, Feb.28 2025

% Gather data from directional ROS, speed and toqrue limit and generate
% plot.

%% Init
figure(1);
set(gcf, 'Position', [100, 100, 600, 600]);

%% Load and plot data for axis symmetrical ROS
clear;
load("data/ros_map_022526_axis_alpha45_1d_high_res.mat");

% Cut end data
gamma = gamma(1:end-1);
max_ros_map = max_ros_map(1:3, 1:end-1); % only using beta = 0, 5, 10, removing 15 for tighter plot

% Duplicate data for other 3 quadrant
max_ros_map_extended = [max_ros_map(:,1:end), max_ros_map(:,1:end), max_ros_map(:,1:end), max_ros_map(:,1:end)];
gamma_extended = linspace(0, 2 * pi * 359/360, length(max_ros_map_extended));

% Rotate plot 45 degrees counter-clockwise to match actual heading angle
% Axis sym is calculate from 0-90, actual gamma = 0 (in paper) is gamma = -45 in matlab 
gamma_extended = gamma_extended - pi/4;

subplot(2, 2, 1);
polarplot(gamma_extended, max_ros_map_extended, 'LineWidth', 1.5);
rlim([0,0.24]);
ax = gca;
ax.FontSize = 14;
ax.FontName = 'Times New Roman';
% Set 0 degrees to be at the top
ax.ThetaZeroLocation = 'top';
title('(a) F_{ROS} -axis', 'FontSize', 14);

%% Load and plot data for mirror symmetrical ROS
clear;
load("data/ros_map_022526_mirror_alpha45_1d_high_res.mat");

% Cut end data
% gamma = gamma(1:end-1);
max_ros_map = max_ros_map(1:3, 1:end); % only using beta = 0, 5, 10, removing 15 for tighter plot

% Duplicate data for other 3 quadrant
max_ros_map_extended = [max_ros_map(:,1:end),max_ros_map(:,end:-1:1),max_ros_map(:,1:end),max_ros_map(:,end:-1:1)];
gamma_extended = linspace(0, 2 * pi * 359/360, length(max_ros_map_extended));

% Rotate plot 90 degrees counter-clockwise to match actual heading angle
% Mirror sym is calculate from 45-134, actual gamma = 0 (in paper) is gamma = -45 in matlab 
gamma_extended = gamma_extended - pi/2;

subplot(2, 2, 3);
polarplot(gamma_extended, max_ros_map_extended, 'LineWidth', 1.5);
rlim([0,0.24]);
ax = gca;
ax.FontSize = 14;
ax.FontName = 'Times New Roman';
% Set 0 degrees to be at the top
ax.ThetaZeroLocation = 'top';
title('(c) F_{ROS} -mirror', 'FontSize', 14);

%% Load and plot data for axis symmetrical torque
clear;
load("data/torque_map_022726_axis.mat");

% Rotate plot 45 degrees counter-clockwise to match actual heading angle
gamma = gamma + pi/4;

subplot(2, 2, 2);
polarplot(gamma, max_torque_map(1:3, :), 'LineWidth', 1.5);
rlim([0, 90]);
rticks([0 30 60 90]);
ax = gca;
ax.FontSize = 14;
ax.FontName = 'Times New Roman';
% Set 0 degrees to be at the top
ax.ThetaZeroLocation = 'top';
title('(b) Longitudinal \tau_{max} -axis', 'FontSize', 14);

%% Load and plot data for mirror symmetrical torque
clear;
load("data/torque_map_022726_mirror.mat");

% Rotate plot 45 degrees counter-clockwise to match actual heading angle
gamma = gamma + pi/4;

subplot(2, 2, 4);
polarplot(gamma, max_torque_map(1:3, :), 'LineWidth', 1.5);
rlim([0, 90]);
rticks([0 30 60 90]);
ax = gca;
ax.FontSize = 14;
ax.FontName = 'Times New Roman';
% Set 0 degrees to be at the top
ax.ThetaZeroLocation = 'top';
title('(d) Longitudinal \tau_{max} -mirror', 'FontSize', 14);