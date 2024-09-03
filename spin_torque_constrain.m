% Created by Tianyi Han, Jun.5 2023

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
params("symmetric_type") = 0;

%% Calculate Hertzian contact model
Fz = (params("m_ball") + params("m_body")) * params("g");   % Applied external load [N]
mu = 1.0;                                                   % Static coefficient of friction
nu = 0.4;                                                   % Poisson ratio of coating under pressure
E = exp(70 * 0.0235 - 0.6403) * 1e6;                        % Young's modulus of coating [Mpa]
h = 12.5e-3;                                                % Coating thickness [m]

tau_z_max = (2 * pi / 5) * mu * Fz * (4 / pi * Fz * (1 - nu^2) * params("r") * h / E)^0.25;
disp("Maximum z torque allowed: " + num2str(tau_z_max));

tau_y_max = mu * Fz * params("r");
disp("Maximum x&y torque allowed: " + num2str(tau_y_max));

%% Loop through motor configs
alpha = linspace(30, 80, 51)./(180/pi);     % [rad]
beta = linspace(0, 90, 91)./(180/pi);       % [rad]
gamma = 0;                                  % [rad]
phi = 0;                                    % [rad]
max_wheel_traction = 9 / 63.5e-3;           % [Nm]

max_torque_map = zeros([length(beta), length(alpha)]);

for alpha_idx = 1:length(alpha)
    for beta_idx = 1:length(beta)

        params("alpha") = alpha(alpha_idx);
        params("beta") = beta(beta_idx);
        params("gamma") = gamma;

        [r_vec_world, R_wheel2world] = rotation_matrix(phi, params);

        % Calculate F_traction in world frame
        ft_wheel_frame = {[], [], [], []};
        ft_world_frame = {[], [], [], []};
        for i = 1:4
            ft_wheel_frame{i} = [max_wheel_traction; 0; 0];
            ft_world_frame{i} = R_wheel2world{i} * ft_wheel_frame{i};
        end

        % Calculate resultant spinning torque
        tau_max = cross(r_vec_world{1}, ft_world_frame{1}) + cross(r_vec_world{2}, ft_world_frame{2}) + ...
            cross(r_vec_world{3}, ft_world_frame{3}) + cross(r_vec_world{4}, ft_world_frame{4});
        tau_max_z = tau_max(3);
        
        max_torque_map(beta_idx, alpha_idx) = tau_max_z;
    end
end

%%  Plot
[A, B] = meshgrid(alpha.*(180/pi), beta.*(180/pi));
figure;
surf(A, B, max_torque_map);
xlabel("\alpha (rad)");
ylabel("\beta (rad)");
zlabel("Max tau_z");
xlim([30, 80]); ylim([0, 90]);
view(-215, 60);
