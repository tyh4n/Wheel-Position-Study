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
params("symmetric_type") = 0;

%% Set speed targets
vx_target = 2;      % x axis target speed [m/s]
vy_target = 0;      % y axis target speed [m/s]
wz_target = 0;      % z axis target angular speed [rad/s]

% Convert to angular speed
wx_target = vx_target / params("r");
wy_target = vy_target / params("r");
w_vec = [wx_target; wy_target; wz_target];

%% Loop through motor configs
alpha = linspace(30, 80, 51)./(180/pi);     % [rad]
beta = linspace(0, 80, 81)./(180/pi);       % [rad]
% With gamma = 45, forward is y-positive, right is x-positive
gamma = 45./(180/pi);                       % [rad]
% Assume in upright position
phi = 0;                                    % [rad]
% Motor rated max speed is 390 rpm
max_wheel_speed = 2.5934;                   % [m/s]

max_speed_map = zeros([length(beta), length(alpha)]);

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
            ft_wheel_frame{i} = [max_wheel_speed; 0; 0];
            ft_world_frame{i} = R_wheel2world{i} * ft_wheel_frame{i};
        end

        % Calculate resultant spinning torque
        tau_max = cross(r_vec_world{1}, ft_world_frame{1}) + cross(r_vec_world{2}, ft_world_frame{2}) + ...
            cross(r_vec_world{3}, ft_world_frame{3}) + cross(r_vec_world{4}, ft_world_frame{4});
        tau_max_z = tau_max(3);
        
        max_speed_map(beta_idx, alpha_idx) = tau_max_z;
    end
end

%%  Plot
[A, B] = meshgrid(alpha.*(180/pi), beta.*(180/pi));
figure;
surf(A, B, max_speed_map);
xlabel("\alpha (rad)");
ylabel("\beta (rad)");
zlabel("Max tau_z");
xlim([30, 80]); ylim([0, 90]);
view(-215, 60);
