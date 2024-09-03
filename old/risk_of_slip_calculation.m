% Created by Tianyi Han, Jun.12 2023

%% Load trajectory from the file
load("data/traj_1.mat");
load("data/rotation_matrix_1.mat");

%% Robot parameters
I_ball = 0.052;         % Ball inertia [kgm^2]
m_ball = 4.0;           % Ball mass[kg]
I_body = 3.0;           % Body inertia [kgm^2]
m_body = 92;            % Body mass [kg] 
r = 0.125;              % Ball radius [m]
l = 0.45;               % Body length (IP) [m]
g = 9.81;               % [m/s^2]
f_ball_arrestor = 0;    % [N]

%% Calculate F_body_ball, F_ground_ball_normal
% Number of steps
num_step = length(tau_sol);

% Vector pointing from ball COM to body COM
l_vec = l * [sin(traj_sol); zeros([1, num_step]); cos(traj_sol)];

% Body and ball trajectory profile
accel_ball = r * [phi_dd_sol + theta_dd_sol; zeros([1, num_step]); zeros([1, num_step])];
alpha_body = [zeros([1, num_step]); phi_dd_sol; zeros([1, num_step])];
omega_body = [zeros([1, num_step]); phi_d_sol; zeros([1, num_step])];
accel_body = accel_ball + cross(alpha_body, l_vec) + cross(cross(omega_body, omega_body), l_vec);

% Force vector time array
f_ball_arrestor_vec  = f_ball_arrestor * [-sin(traj_sol); zeros([1, num_step]); -cos(traj_sol)];
g_vec = g * [zeros([1, num_step]); zeros([1, num_step]); -ones([1, num_step])];
f_body_ball = f_ball_arrestor_vec + [zeros([1, num_step]); zeros([1, num_step]); - ones([1, num_step]) * m_body * g] - m_body * accel_body;
f_ground_ball_normal =  - f_body_ball + [zeros([1, num_step]); zeros([1, num_step]); ones([1, num_step]) * m_ball * g];

%% Calculate stiffness matrix
% Calculaye stiffness matrix at contact frame and body frame
k_wheel_ball = 1e6; % [N/m]
stiffness_matirx_wheel_ball = [[0, 0, 0           ];
                               [0, 0, 0           ];
                               [0, 0, k_wheel_ball]];
stiffness_matrix_ball  = {[], [], [], [], []};
for i = 1:4
    stiffness_matrix_ball{i} = R_wheel2world{i} * stiffness_matirx_wheel_ball * R_wheel2world{i}';
end
% Calculate stiffness matrix at ground-ball contact
k_ground_ball = 1e6; % [N/m]
stiffness_matirx_ground_ball = [[0, 0, 0            ];
                                [0, 0, 0            ];
                                [0, 0, k_ground_ball]];
R_ball2world = eye(3);
stiffness_matrix_ball{5} = R_ball2world * stiffness_matirx_ground_ball * R_ball2world';
% Calculate simplified A matrix
A_matrix = zeros([3, 3]);
for i = 1:5
    A_matrix = A_matrix + stiffness_matrix_ball{i};
end

%% Define variables
% Loop through time steps
for idx = 1:1
    syms ft1 ft2 ft3 ft4; % Traction force at each wheel [N]
    ft = [ft1, ft2, ft3, ft4];
    syms delta_ball_x delta_ball_y delta_ball_z; % Ball linear displacement [m]

    % Calculate F_traction in world frame
    ft_wheel_frame = {[], [], [], []};
    ft_world_frame = {[], [], [], []};
    for i = 1:4
        ft_wheel_frame{i} = [ft(i); 0; 0];
        ft_world_frame{i} = R_wheel2world{i} * ft_wheel_frame{i};
    end

    % Calculate F_normal in world frame
    delta_ball = [delta_ball_x; delta_ball_y; delta_ball_z];
    fn_total = A_matrix * delta_ball;

    % Tau equilibrium
    eqn1 = [0; tau_sol(idx); 0] == cross(r_vec_world{1}, ft_world_frame{1}) + cross(r_vec_world{2}, ...
        ft_world_frame{2}) + cross(r_vec_world{3}, ft_world_frame{3}) + cross(r_vec_world{4}, ft_world_frame{4});

    % Force equilibrium
    ft_total = ft_world_frame{1} + ft_world_frame{2} + ft_world_frame{3} + ft_world_frame{4};
    eqn2 = ft_total + fn_total + f_ground_ball_normal(:, idx) == 0;

    % Traction distribution method constrain
    eqn3 = ft1 == - ft3;

    % Solve multi-variables function
    eqns = [eqn1; eqn2; eqn3];
    S = solve(eqns,[ft1 ft2 ft3 ft4 delta_ball_x delta_ball_y delta_ball_z]);

    % Display solution
    disp('F_traction')
    disp(double(subs([ft1 ft2 ft3 ft4], S)));
    fn1 = stiffness_matrix_ball{1} * delta_ball;
    fn2 = stiffness_matrix_ball{2} * delta_ball;
    fn3 = stiffness_matrix_ball{3} * delta_ball;
    fn4 = stiffness_matrix_ball{4} * delta_ball;
    disp('F_normal')
    disp(double(subs([fn1 fn2 fn3 fn4], S)));
end