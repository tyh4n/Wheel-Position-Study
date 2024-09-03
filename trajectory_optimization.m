% Created by Tianyi Han, Jun.5 2023

%% Define common variables
syms phi_a phi_b real;      % Sech curve magnitude [rad]
phi_scale = [phi_a phi_b]'; 
t_init = 0;                 % Start time [ms]
t_final = 2000;             % End time [ms]
t_middle = t_final / 2;     % Middle time [ms]
t_interval = 10;            % Simulation time resolution [ms]

% Robot parameters
I_ball = 0.052;     % Ball inertia [kgm^2]
m_ball = 4.0;       % Ball mass[kg]
I_body = 3.0;       % Body inertia [kgm^2]
m_body = 92;        % Body mass [kg] 
r = 0.114;          % Ball radius [m]
l = 0.45;           % Body length (IP) [m]
g = 9.81;           % [m/s^2]

% Desired states
theta_final_desired = 4 * pi;   % Desired rotation of the ball [rad]
theta_d_init = 11.2;            % [rad/s]
theta_d_final_desired = 0;      % [rad/s]

% Define weights
w1 = 1; w2 = 1000; w3 = 100;    % Objective function weights
k = 1;                          % The width parameter of the sech curve

% Equation of motion
alpha = I_ball + (m_ball + m_body) * r^2;
beta = m_body * r * l;
gamma = I_body + m_body * l^2;

% Define 'params'
params = [I_ball, m_ball, I_body, m_body, r, l, g, alpha, beta, gamma, t_interval, theta_d_init];

%% Define trajectory
t = t_init : t_interval : t_final;
% Static initial configuration to static final configuration
% traj_phi = phi_a * sech(k * (2 * t - t_middle - t_init) / (t_middle - t_init)) + phi_b * sech(k * (2 * t - t_final - t_middle) / (t_final - t_middle));
% Deceleration trajectory
traj_phi = phi_a * sech(k * (2 * t - t_final - t_init) / (t_final - t_init)) - phi_a * sech(1);
output = phi2tau(traj_phi, params);
tau = output{1};
phi_d = output{3};
phi_dd = output{4};
theta = output{5};
theta_d = output{6};
theta_dd = output{7};

%% Define optimization problem
% Static initial configuration to static final configuration
% F = w1 * sum((t_interval/1000) * tau.^2) + w2 * theta_d(end)^2 + w3 * (theta(end) - theta_final_desired)^2;
% Deceleration trajectory
F = w1 * sum((t_interval/1000) * tau.^2) + w2 * theta_d(end)^2;
obj_fun = matlabFunction(F, 'var', {phi_scale}); 
phi0 = [-7, -1]'.*(pi/180);
phi_lb = [-30, -30]'.*(pi/180);
phi_ub = [30, 30]'.*(pi/180);

%% Call fminunc to solve
options = optimoptions(@fmincon, 'Display', 'iter');
[phi_sol, fval] = fmincon(obj_fun, phi0,[],[],[],[],phi_lb,phi_ub,[],options);

%% Plot trajectory
traj_sol = double(subs(traj_phi, phi_scale, phi_sol));
theta_sol = double(subs(theta, phi_scale, phi_sol));
theta_d_sol = double(subs(theta_d, phi_scale, phi_sol));
tau_sol = double(subs(tau, phi_scale, phi_sol));
figure(1);
subplot(4,1,1);
plot(t./1000, traj_sol./(pi/180));
xlabel('Time (s)');
ylabel('\phi (deg)');
title('Body angle');
subplot(4,1,2);
plot(t./1000, theta_sol);
xlabel('Time (s)');
ylabel('\theta (rad)');
title('Ball angle');
subplot(4,1,3);
plot(t./1000, theta_d_sol);
xlabel('Time (s)');
ylabel('\theta_d (rad/s)');
title('Ball speed');
subplot(4,1,4);
plot(t./1000, tau_sol);
xlabel('Time (s)');
ylabel('\tau (Nm)');
title('Motor torque');
sgtitle('Trajectory Optimization');

%% Save trajectory
phi_d_sol = double(subs(phi_d, phi_scale, phi_sol));
phi_dd_sol = double(subs(phi_dd, phi_scale, phi_sol));
theta_dd_sol = double(subs(theta_dd, phi_scale, phi_sol));
time = t./1000;
save('data/traj_1.mat', 'tau_sol', 'traj_sol', 'phi_d_sol', 'phi_dd_sol', 'theta_dd_sol', 'time');

%% Visualization
sim('visualization',t_final/1000);
% For side view, switch to y up and then front view 

%% Equation of motion
function output = phi2tau(phi, params)
    alpha = params(8);
    beta = params(9);
    gamma = params(10);
    g = params(7);
    r = params(5);
    t_interval = params(11);
    theta_d_init = params(12);
    % Calculate phi dot
    phi_d = (phi - circshift(phi, [0, -1])) ./ (t_interval/1000);
    phi_d(end) = phi_d(end - 1);
    % Calculate phi double dots
    phi_dd = (phi_d - circshift(phi_d, [0, -1])) ./ (t_interval/1000);
    phi_dd(end) = phi_dd(end - 1);
    % Calculate theta double dots
    theta_dd = (beta * g / r * sin(phi) + beta * sin(phi) .* phi_d.^2 - (alpha + gamma + 2 * beta * cos(phi)) .* phi_dd) ./ (alpha + beta * cos(phi));
    % Calculate tau
    tau = alpha * theta_dd + (alpha + beta * cos(phi)) .* phi_dd - beta * sin(phi) .* phi_dd.^2;
    % Calculate theta dot
    theta_d = cumsum(theta_dd) .* (t_interval/1000) + theta_d_init;
    % Calculate theta
    theta = cumsum(theta_d) .* (t_interval/1000);
    % Generate output
    output = {tau, phi, phi_d, phi_dd, theta, theta_d, theta_dd};
end
