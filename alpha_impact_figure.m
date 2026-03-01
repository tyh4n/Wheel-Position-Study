% Created by Tianyi Han, Feb.28 2026

%% Init
figure(1);
set(gcf, 'Position', [100, 100, 600, 600]);

%% Figure 1: alpha vs. loadcapacity
clear;
subplot(3, 1, 1);
alpha = linspace(30, 60, 31);
max_load = 4 * cosd(alpha) * 45;
plot(alpha, max_load, 'LineWidth', 2);
ax = gca;
ax.FontSize = 14;
ax.FontName = 'Times New Roman';
xlabel('\alpha (degrees)');
ylabel('Load_{max} (kg)');
title('(a) Max load capacity', 'FontSize', 14);
grid on;


%% Figure 2: alpha vs. ROS (axis)
clear;
subplot(3, 1, 2);
load('data/ros_map_020226_axis_31a_4b_1g.mat');
plot(alpha./(pi/180), max_ros_map(1:3, :), 'LineWidth', 2);
ylim([0, 0.2]);
ax = gca;
ax.FontSize = 14;
ax.FontName = 'Times New Roman';
xlabel('\alpha (degrees)');
ylabel('F_{ROS}');
lgd = legend('\beta = 0^\circ','\beta = 5^\circ','\beta = 10^\circ');
lgd.NumColumns = 3;
lgd.Location = 'southeast';
title('(b) F_{ROS} -axis', 'FontSize', 14);
grid on;

%% Figure 2: alpha vs. ROS (axis)
clear;
subplot(3, 1, 3);
load('data/ros_map_020226_mirror_31a_4b_1g.mat');
plot(alpha./(pi/180), max_ros_map(1:3, :), 'LineWidth', 2);
ylim([0, 0.2]);
ax = gca;
ax.FontSize = 14;
ax.FontName = 'Times New Roman';
xlabel('\alpha (degrees)');
ylabel('F_{ROS}');
lgd = legend('\beta = 0^\circ','\beta = 5^\circ','\beta = 10^\circ');
lgd.NumColumns = 3;
lgd.Location = 'southeast';
title('(c) F_{ROS} -mirror', 'FontSize', 14);
grid on;