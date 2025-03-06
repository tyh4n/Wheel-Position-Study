% Created by Tianyi Han, Feb.28 2025

% Gather data from directional ROS, speed and toqrue limit and generate
% plot.

%% Init
figure(1);
set(gcf, 'Position', [100, 100, 800, 400]);

%% Load and plot data for axis symmetrical ROS
clear;
load("data/directional_091424_1.mat");

% Duplicate data for other 3 quadrant
max_ros_map_extended = [max_ros_map(1:end-1), max_ros_map(1:end-1), max_ros_map(1:end-1), max_ros_map(1:end)];
gamma_extended = linspace(0, 2 * pi, length(max_ros_map_extended));

% Rotate plot 45 degrees counter-clockwise to match actual heading angle
gamma_extended = gamma_extended + pi/4;

subplot(1, 2, 1);
polarplot(gamma_extended, max_ros_map_extended, 'LineWidth', 2);
rlim([0,0.3]);
ax = gca;
ax.FontSize = 16;
% Set 0 degrees to be at the top
ax.ThetaZeroLocation = 'top';
title('Risk of Slip (Axis)', 'FontSize', 16);

%% Manually adjust ticks location

% Set radial ticks
rticks([0, 0.1, 0.2, 0.3]);

% Remove default radial tick labels
ax.RTickLabels = {};

% Manually add radial tick labels with an offset to move them lower
rtickvalues = ax.RTick; % Get radial tick values
offset = -0.03; % Adjust this value to move labels lower (negative for downward shift)

for i = 1:length(rtickvalues)
% Place text at the correct radial position (0 degrees for alignment)
text(0, rtickvalues(i)+offset, num2str(rtickvalues(i)), ...
'Rotation', 0, ... % No rotation (vertical)
'HorizontalAlignment', 'center', ... % Center horizontally
'VerticalAlignment', 'bottom', ... % Align at the bottom
'FontSize', ax.FontSize); % Match font size
end

%% Load and plot data for mirror symmetrical ROS
clear;
load("data/directional_022825_mirror.mat");

% Duplicate data for other 3 quadrant
max_ros_map_extended = [max_ros_map(1:end-1),max_ros_map(end:-1:2),max_ros_map(1:end-1),max_ros_map(end:-1:1)];
gamma_extended = linspace(0, 2 * pi, length(max_ros_map_extended));

% Rotate plot 45 degrees counter-clockwise to match actual heading angle
gamma_extended = gamma_extended + pi/4;

subplot(1, 2, 2);
polarplot(gamma_extended, max_ros_map_extended, 'LineWidth', 2);
rlim([0,0.3]);
ax = gca;
ax.FontSize = 16;
% Set 0 degrees to be at the top
ax.ThetaZeroLocation = 'top';
title('Risk of Slip (Mirror)', 'FontSize', 16);

%% Manually adjust ticks location

% Set radial ticks
rticks([0, 0.1, 0.2, 0.3]);

% Remove default radial tick labels
ax.RTickLabels = {};

% Manually add radial tick labels with an offset to move them lower
rtickvalues = ax.RTick; % Get radial tick values
offset = -0.03; % Adjust this value to move labels lower (negative for downward shift)

for i = 1:length(rtickvalues)
% Place text at the correct radial position (0 degrees for alignment)
text(0, rtickvalues(i)+offset, num2str(rtickvalues(i)), ...
'Rotation', 0, ... % No rotation (vertical)
'HorizontalAlignment', 'center', ... % Center horizontally
'VerticalAlignment', 'bottom', ... % Align at the bottom
'FontSize', ax.FontSize); % Match font size
end

%%
figure(2);
set(gcf, 'Position', [200, 200, 800, 800]);

%% Load and plot data for axis symmetrical velocity
clear;

subplot(2, 2, 1);

load("data/max_velocity_map_beta0_axis.mat");
% Rotate plot 45 degrees counter-clockwise to match actual heading angle
gamma = gamma + pi/4; 
polarplot(gamma, max_velocity_map, 'LineWidth', 2);
hold on;

load("data/max_velocity_map_beta5_axis.mat");
gamma = gamma + pi/4; 
polarplot(gamma, max_velocity_map, 'LineWidth', 2);

load("data/max_velocity_map_022825_axis.mat");
gamma = gamma + pi/4;
polarplot(gamma, max_velocity_map, 'LineWidth', 2);

load("data/max_velocity_map_beta15_axis.mat");
gamma = gamma + pi/4;
% polarplot(gamma, max_velocity_map, 'LineWidth', 2);

rlim([0, 7]);
rticks(0:2:6);
ax = gca;
ax.FontSize = 16;
% Set 0 degrees to be at the top
ax.ThetaZeroLocation = 'top';
title('Max Velocity (Axis)', 'FontSize', 16);

%% Manually adjust ticks location

% Set radial ticks
rticks([0, 2, 4, 6]);

% Remove default radial tick labels
ax.RTickLabels = {};

% Manually add radial tick labels with an offset to move them lower
rtickvalues = ax.RTick; % Get radial tick values
offset = -0.7; % Adjust this value to move labels lower (negative for downward shift)

for i = 1:length(rtickvalues)
% Place text at the correct radial position (0 degrees for alignment)
text(0, rtickvalues(i)+offset, num2str(rtickvalues(i)), ...
'Rotation', 0, ... % No rotation (vertical)
'HorizontalAlignment', 'center', ... % Center horizontally
'VerticalAlignment', 'bottom', ... % Align at the bottom
'FontSize', ax.FontSize); % Match font size
end

%% Load and plot data for mirror symmetrical velocity
clear;

subplot(2, 2, 2);

load("data/max_velocity_map_beta0_mirror.mat");
% Rotate plot 45 degrees counter-clockwise to match actual heading angle
gamma = gamma + pi/4;
polarplot(gamma, max_velocity_map, 'LineWidth', 2);
hold on;

load("data/max_velocity_map_beta5_mirror.mat");
gamma = gamma + pi/4;
polarplot(gamma, max_velocity_map, 'LineWidth', 2);

load("data/max_velocity_map_022825_mirror.mat");
gamma = gamma + pi/4;
polarplot(gamma, max_velocity_map, 'LineWidth', 2);

load("data/max_velocity_map_beta15_mirror.mat");
gamma = gamma + pi/4;
% polarplot(gamma, max_velocity_map, 'LineWidth', 2);

rlim([0, 7]);
rticks(0:2:6);
ax = gca;
ax.FontSize = 16;
% Set 0 degrees to be at the top
ax.ThetaZeroLocation = 'top';
title('Max Velocity (Mirror)', 'FontSize', 16);

%% Manually adjust ticks location

% Set radial ticks
rticks([0, 2, 4, 6]);

% Remove default radial tick labels
ax.RTickLabels = {};

% Manually add radial tick labels with an offset to move them lower
rtickvalues = ax.RTick; % Get radial tick values
offset = -0.7; % Adjust this value to move labels lower (negative for downward shift)

for i = 1:length(rtickvalues)
% Place text at the correct radial position (0 degrees for alignment)
text(0, rtickvalues(i)+offset, num2str(rtickvalues(i)), ...
'Rotation', 0, ... % No rotation (vertical)
'HorizontalAlignment', 'center', ... % Center horizontally
'VerticalAlignment', 'bottom', ... % Align at the bottom
'FontSize', ax.FontSize); % Match font size
end

%% Load and plot data for axis symmetrical torque
clear;

subplot(2, 2, 3);

load("data/max_torque_map_beta0_axis.mat");
% Rotate plot 45 degrees counter-clockwise to match actual heading angle
gamma = gamma + pi/4;
polarplot(gamma, max_torque_map, 'LineWidth', 2);
hold on;

load("data/max_torque_map_beta5_axis.mat");
gamma = gamma + pi/4;
polarplot(gamma, max_torque_map, 'LineWidth', 2);

load("data/max_torque_map_022725_axis.mat");
gamma = gamma + pi/4;
polarplot(gamma, max_torque_map, 'LineWidth', 2);

load("data/max_torque_map_beta15_axis.mat");
gamma = gamma + pi/4;
% polarplot(gamma, max_torque_map, 'LineWidth', 2);

rlim([0, 90]);
rticks(0:30:90);
ax = gca;
ax.FontSize = 16;
% Set 0 degrees to be at the top
ax.ThetaZeroLocation = 'top';
title('Max Torque (Axis)', 'FontSize', 16);

%% Manually adjust ticks location

% Set radial ticks
rticks([0, 30, 60, 90]);

% Remove default radial tick labels
ax.RTickLabels = {};

% Manually add radial tick labels with an offset to move them lower
rtickvalues = ax.RTick; % Get radial tick values
offset = -10; % Adjust this value to move labels lower (negative for downward shift)

for i = 1:length(rtickvalues)
% Place text at the correct radial position (0 degrees for alignment)
text(0, rtickvalues(i)+offset, num2str(rtickvalues(i)), ...
'Rotation', 0, ... % No rotation (vertical)
'HorizontalAlignment', 'center', ... % Center horizontally
'VerticalAlignment', 'bottom', ... % Align at the bottom
'FontSize', ax.FontSize); % Match font size
end

%% Load and plot data for mirror symmetrical torque
clear;

subplot(2, 2, 4);

load("data/max_torque_map_beta0_axis.mat");
% Rotate plot 45 degrees counter-clockwise to match actual heading angle
gamma = gamma + pi/4;
polarplot(gamma, max_torque_map, 'LineWidth', 2);
hold on;

load("data/max_torque_map_beta5_mirror.mat");
gamma = gamma + pi/4;
polarplot(gamma, max_torque_map, 'LineWidth', 2);

load("data/max_torque_map_022725_mirror.mat");
gamma = gamma + pi/4;
polarplot(gamma, max_torque_map, 'LineWidth', 2);

load("data/max_torque_map_beta15_mirror.mat");
gamma = gamma + pi/4;
% polarplot(gamma, max_torque_map, 'LineWidth', 2);

rlim([0, 90]);
rticks(0:30:90);
ax = gca;
ax.FontSize = 16;
% Set 0 degrees to be at the top
ax.ThetaZeroLocation = 'top';
title('Max Torque (Mirror)', 'FontSize', 16);

%% Manually adjust ticks location

% Set radial ticks
rticks([0, 30, 60, 90]);

% Remove default radial tick labels
ax.RTickLabels = {};

% Manually add radial tick labels with an offset to move them lower
rtickvalues = ax.RTick; % Get radial tick values
offset = -10; % Adjust this value to move labels lower (negative for downward shift)

for i = 1:length(rtickvalues)
% Place text at the correct radial position (0 degrees for alignment)
text(0, rtickvalues(i)+offset, num2str(rtickvalues(i)), ...
'Rotation', 0, ... % No rotation (vertical)
'HorizontalAlignment', 'center', ... % Center horizontally
'VerticalAlignment', 'bottom', ... % Align at the bottom
'FontSize', ax.FontSize); % Match font size
end