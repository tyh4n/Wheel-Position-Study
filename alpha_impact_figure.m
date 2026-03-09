% Created by Tianyi Han, Feb.28 2026

%% Init
figure(1);
set(gcf, 'Position', [100, 100, 600, 700]);


%% Figure 1: alpha vs. ROS (axis)
clear;
subplot(4, 1, 1);
load('data/ros_map_020226_axis_31a_4b_1g.mat');
plot(alpha./(pi/180), max_ros_map(1:3, :), 'LineWidth', 1.5);
ylim([0, 0.2]);
ax = gca;
ax.FontSize = 14;
ax.FontName = 'Times New Roman';
xlabel('\alpha (degrees)');
ylabel('ROS_{max}');
% lgd = legend('\beta = 0^\circ','\beta = 5^\circ','\beta = 10^\circ');
% lgd.NumColumns = 3;
% lgd.Location = 'southeast';
title('(a) ROS_{max} -axis', 'FontSize', 14);
grid on;

%% Figure 2: alpha vs. ROS (axis)
clear;
subplot(4, 1, 2);
load('data/ros_map_020226_mirror_31a_4b_1g.mat');
plot(alpha./(pi/180), max_ros_map(1:3, :), 'LineWidth', 1.5);
ylim([0, 0.2]);
ax = gca;
ax.FontSize = 14;
ax.FontName = 'Times New Roman';
xlabel('\alpha (degrees)');
ylabel('ROS_{max}');
% lgd = legend('\beta = 0^\circ','\beta = 5^\circ','\beta = 10^\circ');
% lgd.NumColumns = 3;
% lgd.Location = 'southeast';
title('(b) ROS_{max} -mirror', 'FontSize', 14);
grid on;

%% Figure 3: alpha vs. loadcapacity
clear;
subplot(4, 1, 3);
alpha = linspace(30, 60, 31);
max_load = 4 * cosd(alpha) * 45;
plot(alpha, max_load, 'LineWidth', 1.5);
ylim([80, 160]);
ax = gca;
ax.FontSize = 14;
ax.FontName = 'Times New Roman';
xlabel('\alpha (degrees)');
ylabel('Load_{max} (kg)');
title('(c) Load_{max}', 'FontSize', 14);
grid on;

%% Figure 4: alpha vs. rotation torque
clear;
subplot(4, 1, 4);
load('data/rotation_map_020226.mat');
plot(alpha./(pi/180), max_torque_map(1:3, :), 'LineWidth', 1.5);
yline(31.9, 'Color', 'r', 'LineWidth', 1.5); 
ylim([20, 45]);
ax = gca;
ax.FontSize = 14;
ax.FontName = 'Times New Roman';
xlabel('\alpha (degrees)');
ylabel('\tau_{z,max} (Nm)');
% lgd = legend('\beta = 0^\circ','\beta = 5^\circ','\beta = 10^\circ');
% lgd.NumColumns = 3;
% lgd.Location = 'southoutside';
title('(d) \tau_{z,max}', 'FontSize', 14);
grid on;