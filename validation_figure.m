%% Load and plot simulation data
clear;
load("data/ros_map_022526_mirror_alpha45_1d_high_res.mat");

% Cut end data
% gamma = gamma(1:end-1);
max_ros_map = max_ros_map(3, 1:end); % only using beta = 0, 5, 10, removing 15 for tighter plot

% Duplicate data for other 3 quadrant
max_ros_map_extended = [max_ros_map(:,1:end),max_ros_map(:,end:-1:1),max_ros_map(:,1:end),max_ros_map(:,end:-1:1)];
gamma_extended = linspace(0, 2 * pi * 359/360, length(max_ros_map_extended));

% Rotate plot 90 degrees counter-clockwise to match actual heading angle
% Mirror sym is calculate from 45-134, actual gamma = 0 (in paper) is gamma = -45 in matlab 
gamma_extended = gamma_extended - pi/2;

figure(1);
set(gcf, 'Position', [100, 100, 600, 600]);
polarplot(gamma_extended, max_ros_map_extended, 'LineWidth', 2);
rlim([0,0.3]);
ax = gca;
ax.FontSize = 20;
ax.FontName = 'Times New Roman';
% Set 0 degrees to be at the top
ax.ThetaZeroLocation = 'top';

%% --- Empirical Time-Series Overlay (Jittered) ---

% 1. Load the processed data table
load('data/braking_processed_results.mat', 'results_table');

% 2. Filter the table to keep only the valid moves
valid_data = results_table(results_table.Is_Kept == 1, :);

hold on;

% Define angles and colors
angles = [0, pi/2, pi/4]; 
gem_palette = orderedcolors('gem'); 
colors = {gem_palette(1,:), gem_palette(2,:), gem_palette(3,:)};

% --- NEW: Jitter Setting ---
% Set the total width of the scatter spread (in degrees)
jitter_deg = 2; 
jitter_rad = jitter_deg * (pi/180);

% 3. Loop through all valid moves to plot their full arrays
for i = 1:height(valid_data)
    
    ros_array = valid_data.ROS_TimeSeries_Array{i};
    sec = valid_data.Section(i);
    
    % Create base theta array
    theta_base = repmat(angles(sec), size(ros_array));
    
    % Apply random angular jitter to spread the points horizontally
    % rand() - 0.5 generates numbers between -0.5 and 0.5
    theta_jittered = theta_base + (rand(size(ros_array)) - 0.5) * jitter_rad;
    
    % Scatter plot the jittered cloud
    polarscatter(theta_jittered, ros_array, 10, colors{sec}, 'filled', ...
                 'MarkerFaceAlpha', 0.10, 'MarkerEdgeColor', 'none');
end

% --- Legend and Title Setup ---

% 1. Dummy plot for the Simulation Curve (Black Line)
h_sim = polarplot(NaN, NaN, '-', 'Color', colors{1}, 'LineWidth', 1.5);

% 2. Dummy plots for the Scatter Data (Fixed the strokes)
% 'LineStyle', 'none' and 'MarkerEdgeColor', 'none' ensure no rogue borders
h1 = polarplot(NaN, NaN, 'Marker', 'o', 'LineStyle', 'none', ...
               'MarkerFaceColor', colors{1}, 'MarkerEdgeColor', 'none');
h2 = polarplot(NaN, NaN, 'Marker', 'o', 'LineStyle', 'none', ...
               'MarkerFaceColor', colors{2}, 'MarkerEdgeColor', 'none');
h3 = polarplot(NaN, NaN, 'Marker', 'o', 'LineStyle', 'none', ...
               'MarkerFaceColor', colors{3}, 'MarkerEdgeColor', 'none');

% 3. Create the legend with the Simulation included
% Reordered to match your specific bottom layout: [Sim, 0, 90, 45]
legend([h_sim, h1, h3, h2], ...
       {'Sim', '\gamma=0^\circ', '\gamma=45^\circ', '\gamma=90^\circ'}, ...
       'Location', 'southoutside', 'Orientation', 'horizontal');

% 4. Add the Title
title('Simulation vs. Experimental Braking ROS', 'FontSize', 20, 'FontWeight', 'bold');