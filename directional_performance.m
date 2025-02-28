% Created by Tianyi Han, Sep.14 2024

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
params("m_body") = 92;        % Body mass [kg] 
params("r") = 0.114;          % Ball radius [m]
params("l") = 0.45;           % Body length (IP) [m]
params("g") = 9.81;           % [m/s^2]
% [NOTE] if parameters above is changed, rerun trajectory optimization

params("f_ball_arrestor") = 0;  % [N]
params("k_wheel_ball") = 1e6;   % [N/m]
% Torque distribution: 0 for conventional, 1 for equal ros
params("torque_distribution") = 0;
% Symmetric type: 0 for center symmetric, 1 for mirror symmetric
params("symmetric_type") = 1;

%% Loop through motor configs
alpha = 45./(180/pi);      % [rad]
beta = 10./(180/pi);      % [rad]
gamma = linspace(0, 90, 31)./(180/pi);       % [rad]

max_ros_map = zeros(1, length(gamma));

for gamma_idx = 1:length(gamma)
    disp("-------------------------------");
    disp("Current config: ")
    disp([alpha, beta, gamma(gamma_idx).*(180/pi)]);

    params("alpha") = alpha;
    params("beta") = beta;
    params("gamma") = gamma(gamma_idx);

    [max_ros] = ros_calculation(params);
    max_ros_map(gamma_idx) = max_ros;
    disp(['Max risk of slip: ', num2str(max_ros)]);
end


%% Save data 
save('data/directional_022825_mirror.mat', 'gamma', 'max_ros_map');

%% Plot results

if (params("symmetric_type") == 0)
    max_ros_map_extended = [max_ros_map(1:end-1), max_ros_map(1:end-1), max_ros_map(1:end-1), max_ros_map(1:end)];
    gamma_extended = linspace(0, 2 * pi, length(max_ros_map_extended));
    figure(1);
    polarplot(gamma_extended, max_ros_map_extended, 'LineWidth', 2);
    rlim([0,0.3]);
    ax = gca;
    ax.FontSize = 16;
    title('Four-wheel Axis Symmetrical', 'FontSize', 16);
elseif (params("symmetric_type") == 1)
    max_ros_map_extended = [max_ros_map(1:end-1),max_ros_map(end:-1:2),max_ros_map(1:end-1),max_ros_map(end:-1:1)];
    gamma_extended = linspace(0, 2 * pi, length(max_ros_map_extended));
    figure(1);
    polarplot(gamma_extended, max_ros_map_extended, 'LineWidth', 2);
    rlim([0,0.3]);
    ax = gca;
    ax.FontSize = 16;
    title('Four-wheel Mirror Symmetrical', 'FontSize', 16);
end

%% Show total running time
toc;


