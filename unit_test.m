% Created by Tianyi Han, Jun.20 2023

%% Robot parameters
% Create empty dictionary
params = dictionary(string([]), []);
% Set values to attributes
params("I_ball") = 0.052;     % Ball inertia [kgm^2]
params("m_ball") = 4.0;       % Ball mass[kg]
params("I_body") = 3.0;       % Body inertia [kgm^2]
params("m_body") = 92;        % Body mass [kg] 
params("r") = 0.125;          % Ball radius [m]
params("l") = 0.45;           % Body length (IP) [m]
params("g") = 9.81;           % [m/s^2]
% [NOTE] if parameters above is changed, rerun trajectory optimization

params("f_ball_arrestor") = 0;  % [N]
params("k_wheel_ball") = 1e6;   % [N/m]
params("k_ground_ball") = 1e6;  % [N/m]
% Torque distribution: 0 for conventional, 1 for equal ros
params("torque_distribution") = 0;
% Symmetric type: 0 for center symmetric, 1 for mirror symmetric
params("symmetric_type") = 1;

%% Set motor configs
alpha = 45 * (pi/180);   % [rad]
beta = -10 * (pi/180);    % [rad]
gamma = 45 * (pi/180);   % [rad]

params("alpha") = alpha;
params("beta") = beta;
params("gamma") = gamma;

%% Test rotation matrix generation
phi = 15./(180/pi);        % [rad]
[r_vec_world, R_wheel2world] = rotation_matrix(phi, params);

%% Test risk of slip calculation
[max_ros] = ros_calculation(params);
disp(['Max risk of slip: ', num2str(max_ros)]);