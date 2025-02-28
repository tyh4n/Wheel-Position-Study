% Created by Tianyi Han, Feb.28 2025

% Gather data from directional ROS, speed and toqrue limit and generate
% plot.

%% Init
figure(1);
set(gcf, 'Position', [100, 100, 600, 900]);

%% Load and plot data for axis symmetrical ROS
clear;
load("data/directional_091424_1.mat");

% Duplicate data for other 3 quadrant
max_ros_map_extended = [max_ros_map(1:end-1), max_ros_map(1:end-1), max_ros_map(1:end-1), max_ros_map(1:end)];
gamma_extended = linspace(0, 2 * pi, length(max_ros_map_extended));

% Rotate plot 45 degrees counter-clockwise to match actual heading angle
gamma_extended = gamma_extended + pi/4;

subplot(3, 2, 1);
polarplot(gamma_extended, max_ros_map_extended, 'LineWidth', 2);
rlim([0,0.3]);
ax = gca;
ax.FontSize = 16;
% Set 0 degrees to be at the top
ax.ThetaZeroLocation = 'top';
title('ROS (axis)', 'FontSize', 16);

%% Load and plot data for mirror symmetrical ROS
clear;
load("data/directional_022825_mirror.mat");

% Duplicate data for other 3 quadrant
max_ros_map_extended = [max_ros_map(1:end-1),max_ros_map(end:-1:2),max_ros_map(1:end-1),max_ros_map(end:-1:1)];
gamma_extended = linspace(0, 2 * pi, length(max_ros_map_extended));

% Rotate plot 45 degrees counter-clockwise to match actual heading angle
gamma_extended = gamma_extended + pi/4;

subplot(3, 2, 2);
polarplot(gamma_extended, max_ros_map_extended, 'LineWidth', 2);
rlim([0,0.3]);
ax = gca;
ax.FontSize = 16;
% Set 0 degrees to be at the top
ax.ThetaZeroLocation = 'top';
title('ROS (mirror)', 'FontSize', 16);

%% Load and plot data for axis symmetrical velocity
clear;
load("data/max_velocity_map_022825_axis.mat");

% Rotate plot 45 degrees counter-clockwise to match actual heading angle
gamma = gamma + pi/4;

subplot(3, 2, 3);
polarplot(gamma, max_velocity_map, 'LineWidth', 2);
rlim([0, 8]);
ax = gca;
ax.FontSize = 16;
% Set 0 degrees to be at the top
ax.ThetaZeroLocation = 'top';
title('max velocity (axis)', 'FontSize', 16);

%% Load and plot data for mirror symmetrical velocity
clear;
load("data/max_velocity_map_022825_mirror.mat");

% Rotate plot 45 degrees counter-clockwise to match actual heading angle
gamma = gamma + pi/4;

subplot(3, 2, 4);
polarplot(gamma, max_velocity_map, 'LineWidth', 2);
rlim([0, 8]);
ax = gca;
ax.FontSize = 16;
% Set 0 degrees to be at the top
ax.ThetaZeroLocation = 'top';
title('max velocity (mirror)', 'FontSize', 16);

%% Load and plot data for axis symmetrical torque
clear;
load("data/max_torque_map_022725_axis.mat");

% Rotate plot 45 degrees counter-clockwise to match actual heading angle
gamma = gamma + pi/4;

subplot(3, 2, 5);
polarplot(gamma, max_torque_map, 'LineWidth', 2);
rlim([0, 100]);
ax = gca;
ax.FontSize = 16;
% Set 0 degrees to be at the top
ax.ThetaZeroLocation = 'top';
title('max torque (axis)', 'FontSize', 16);

%% Load and plot data for mirror symmetrical torque
clear;
load("data/max_torque_map_022725_mirror.mat");

% Rotate plot 45 degrees counter-clockwise to match actual heading angle
gamma = gamma + pi/4;

subplot(3, 2, 6);
polarplot(gamma, max_torque_map, 'LineWidth', 2);
rlim([0, 100]);
ax = gca;
ax.FontSize = 16;
% Set 0 degrees to be at the top
ax.ThetaZeroLocation = 'top';
title('max torque (mirror)', 'FontSize', 16);