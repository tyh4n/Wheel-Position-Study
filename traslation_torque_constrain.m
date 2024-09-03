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
% Symmetric type: 0 for center symmetric, 1 for mirror symmetric
params("symmetric_type") = 1;

%% Loop through motor configs
alpha = 45./(180/pi);      % [rad]
beta = 10./(180/pi);      % [rad]
gamma = linspace(45, 45, 1)./(180/pi);     % [rad]
phi = 0;                                    % [rad]
max_wheel_force = 18 / 0.0635;              % [N] Assuming peak motor torque

max_torque_map = zeros([length(gamma),1]);

for gamma_idx = 1:length(gamma)

    params("alpha") = alpha;
    params("beta") = beta;
    params("gamma") = gamma(gamma_idx);

    [r_vec_world, R_wheel2world] = rotation_matrix(phi, params);

    % Calculate F_traction in world frame
    ft_wheel_frame = {[], [], [], []};
    ft_world_frame = {[], [], [], []};
    for i = 1:4
        if (i<=2)
            ft_wheel_frame{i} = [max_wheel_force; 0; 0];
        else 
            ft_wheel_frame{i} = [-max_wheel_force; 0; 0];
        end
        ft_world_frame{i} = R_wheel2world{i} * ft_wheel_frame{i};
    end

    % Calculate resultant spinning torque
    tau_max = cross(r_vec_world{1}, ft_world_frame{1}) + cross(r_vec_world{2}, ft_world_frame{2}) + ...
        cross(r_vec_world{3}, ft_world_frame{3}) + cross(r_vec_world{4}, ft_world_frame{4});
    tau_max_z = tau_max(1);
    
    max_torque_map(gamma_idx) = tau_max_z;
end


%%  Plot
figure;

theta = linspace(0, 360, 37)./(180/pi);
polarplot(theta,max_torque_map)

