% Created by Tianyi Han, Sep.14 2024

% Load the saved data file
file_name = "data/ros_map_091424_mirror.mat";
load(file_name); 

% Initialize
params = dictionary(string([]), []);
params("r") = 0.114;          % Ball radius [m]
% Symmetric type: 0 for center symmetric, 1 for mirror symmetric
params("symmetric_type") = 1;
phi = 0;                      % Tilt angle [rad]
ft = -100;                     % Traction force [N]
objective_map = ones(size(max_ros_map)) * 1.8;

% Define weights
w1 = 5;
w2 = 1;
w3 = 0.25;

% Loop through the alpha and beta values
for alpha_idx = 1:length(alpha)
    for beta_idx = 1:length(beta)
        params("alpha") = alpha(alpha_idx);
        params("beta") = - beta(beta_idx);
        params("gamma") = 45./(180/pi);      % [rad];

        % Add the current ROS into the new map
        objective_map(beta_idx, alpha_idx) = objective_map(beta_idx, alpha_idx) + w1 * max_ros_map(beta_idx, alpha_idx);

        % Add load capacity
        objective_map(beta_idx, alpha_idx) = objective_map(beta_idx, alpha_idx) - w2 * cos(alpha(alpha_idx));

        % Add forward torque 
        [r_vec_world, R_world2wheel] = rotation_matrix(phi, params);
        f_wheel_frame = [ft, 0, 0]';
        f_world_frame = R_world2wheel{1} * f_wheel_frame;
        total_torque = cross(r_vec_world{1}, f_world_frame);
        objective_map(beta_idx, alpha_idx) = objective_map(beta_idx, alpha_idx) - w3 * total_torque(2);
    end
end

% Create a meshgrid for alpha and beta
[AlphaGrid, BetaGrid] = meshgrid(alpha .* (180/pi), beta .* (180/pi));

% Plot the surface of the new max_ros_map
figure;
surf(AlphaGrid, BetaGrid, objective_map);
xlabel('Alpha (degrees)', 'FontSize', 16);
ylabel('Beta (degrees)', 'FontSize', 16);
zlabel('Max Risk of Slip', 'FontSize', 16);
zlim([0, 2]);
shading interp;  % smoothens the surface appearance
colorbar;
ax = gca;
ax.FontSize = 14;
title('Four-wheel Axis Symmetrical', 'FontSize', 16);
set(gcf, 'Position', [817, 612, 560, 350]); 
view(-45, 45);
