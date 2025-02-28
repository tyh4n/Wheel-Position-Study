% Created by Tianyi Han, Aug.15 2023

%% Initialize
tic;
clear;

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
params("symmetric_type") = 0;

%% Loop through motor configs
% Set alpha, beta, gamma angle
alpha = 45./(180/pi);                       % [rad]
beta = 10./(180/pi);                        % [rad]
gamma = linspace(0, 360, 361)./(180/pi);    % [rad]
% Assume in upright position
phi = 0;                                    % [rad]
% Motor rated max torque is 20 Nm
tau_motor_lim = 20;                 % [Nm]
% Torque increment step
tau_step = 0.25;

max_torque_map = zeros(1, length(gamma),1);

for gamma_idx = 1:length(gamma)
    % Init
    tau_motor_max = 0;
    tau_ball_tar = 0;

    while (tau_motor_max <= tau_motor_lim)
        tau_ball_tar = tau_ball_tar + tau_step;

        % Calculet ball torque in world frame
        tau_ball_world = [0; tau_ball_tar; 0];

        params("alpha") = alpha;
        params("beta") = beta;
        params("gamma") = gamma(gamma_idx);
    
        % Calculate roatation matrix
        [r_vec_world, R_wheel2world] = rotation_matrix(phi, params);
    
        syms ft1 ft2 ft3 ft4; % Traction force at each wheel [N]
        ft = [ft1, ft2, ft3, ft4];

        % Calculate F_traction in world frame
        ft_wheel_frame = {[], [], [], []};
        ft_world_frame = {[], [], [], []};
        for i = 1:4
            ft_wheel_frame{i} = [ft(i); 0; 0];
            ft_world_frame{i} = R_wheel2world{i} * ft_wheel_frame{i};
        end

        % F_normal in stationary case is simply the body weight
        fn_total = [0; 0; -params("m_body") * params("g")];
        f_ground_ball_normal = [0; 0; (params("m_body") + params("m_ball"))* params("g")];

        % Tau equilibrium
        eqn1 = tau_ball_world == cross(r_vec_world{1}, ft_world_frame{1}) + cross(r_vec_world{2}, ...
            ft_world_frame{2}) + cross(r_vec_world{3}, ft_world_frame{3}) + cross(r_vec_world{4}, ft_world_frame{4});

        % Traction distribution method constrain
        eqn3 = ft1 == - ft3;
        
        % Solve multi-variables function
        eqns = [eqn1; eqn3];
        S = solve(eqns,[ft1 ft2 ft3 ft4]);
        ft_sol = double(subs(ft, S));

        % Calculate max motor torque
        tau_motor_max = max(max(ft_sol(1), ft_sol(2)), max(ft_sol(3), ft_sol(4))) * 0.0625;
    end

    max_torque_map(gamma_idx) = tau_ball_tar;
end

%% Save data 
save('data/max_torque_map_022725_axis.mat', 'alpha', 'beta', 'gamma', 'max_torque_map');

%%  Plot
figure(1);
polarplot(gamma, max_torque_map, 'LineWidth', 2);
% rlim([0,0.3]);
ax = gca;
ax.FontSize = 16;
title('Four-wheel Axis Symmetrical', 'FontSize', 16);

%% Show total running time
toc;
