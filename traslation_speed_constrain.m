% Created by Tianyi Han, Aug.15 2023

%% Robot parameters
% Create empty dictionary
params = dictionary(string([]), []);
% Set values to attributes
params("I_ball") = 0.052;     % Ball inertia [kgm^2]
params("m_ball") = 4.0;       % Ball mass[kg]
params("I_body") = 3.0;       % Body inertia [kgm^2]
params("I_body_z") = 1.0;     % Body inertia spinning [kgm^2]
params("m_body") = 92;        % Body mass [kg] 
params("r") = 0.114;          % Ball radius [m]
params("l") = 0.45;           % Body length (IP) [m]
params("g") = 9.81;           % [m/s^2]
% Symmetric type: 0 for axis symmetric, 1 for mirror symmetric
params("symmetric_type") = 1;

%% Loop through motor configs
% Set alpha, beta, gamma angle
alpha = 45./(180/pi);                         % [rad]
beta = linspace(0, 15, 4)./(180/pi);          % [rad]
gamma = linspace(0, 360, 361)./(180/pi);      % [rad]
% Assume in upright position
phi = 0;                                    % [rad]
% Motor rated max angular velocity is 390 rpm
v_motor_lim = 2.553;                 % [m/s]
% Velocity increment step
v_step = 1e-2;

max_velocity_map = zeros(length(beta), length(gamma));

for beta_idx = 1:length(beta)
    for gamma_idx = 1:length(gamma)
        % Init
        v_motor_max = 0;
        v_ball_tar = 3;
    
        while (v_motor_max <= v_motor_lim) 
            v_ball_tar = v_ball_tar + v_step;
    
            params("alpha") = alpha;
            params("beta") = beta(beta_idx);
            params("gamma") = gamma(gamma_idx);
    
            [r_vec_world, R_wheel2world] = rotation_matrix(phi, params);
    
            % Calculet ball angular velocity in world frame
            w_ball = [-v_ball_tar / params("r"); 0; 0];
    
            % Calculate wheel unit vector in world frame
            uv_wheel_frame = {[], [], [], []};
            uv_world_frame = {[], [], [], []};
            for i = 1:4
                uv_wheel_frame{i} = [1; 0; 0];
                uv_world_frame{i} = R_wheel2world{i} * uv_wheel_frame{i};
            end
    
            % Calculate motor velocity in world frame
            v_motor = {[], [], [], []};
            for i = 1:4
                v_contact = cross(w_ball, r_vec_world{i});
                v_motor{i} = norm(dot(v_contact, uv_world_frame{i}));
            end
    
            % Calculate max motor velocity
            v_motor_max = max(max(v_motor{1}, v_motor{2}), max(v_motor{3}, v_motor{4}));
        end
        

        max_velocity_map(beta_idx,gamma_idx) = v_ball_tar;
    end
end

%% Save data 
save('data/v_map_022626_mirror.mat', 'alpha', 'beta', 'gamma', 'max_velocity_map');

%%  Plot
figure(1);
polarplot(gamma, max_velocity_map, 'LineWidth', 2);
% rlim([0,0.3]);
ax = gca;
ax.FontSize = 16;
title('Four-wheel Axis Symmetrical', 'FontSize', 16);
