% Created by Tianyi Han, Jun.19 2023

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
params("symmetric_type") = 0;

%% Loop through motor configs
alpha = linspace(30, 60, 7)./(180/pi);      % [rad]
beta = linspace(0, 15, 7)./(180/pi);      % [rad]
gamma = linspace(0, 90, 13)./(180/pi);       % [rad]

max_ros_map = zeros([length(beta), length(alpha)]);

for alpha_idx = 1:length(alpha)
    for beta_idx = 1:length(beta)
        for gamma_idx = 1:length(gamma)
            disp("-------------------------------");
            disp("Current config: ")
            disp([alpha(alpha_idx).*(180/pi), beta(beta_idx).*(180/pi), gamma(gamma_idx).*(180/pi)]);

            params("alpha") = alpha(alpha_idx);
            params("beta") = beta(beta_idx);
            params("gamma") = gamma(gamma_idx);

            [max_ros] = ros_calculation(params);
            max_ros_map(beta_idx, alpha_idx) = max(max_ros_map(beta_idx, alpha_idx), max_ros);
            disp(['Max risk of slip: ', num2str(max_ros)]);
        end
    end
end

%% Save data 
save('data/ros_map_091424_axis.mat', 'alpha', 'beta', 'gamma', 'max_ros_map');

%% Plot results
% Plot for center symmetric configuration
[A, B] = meshgrid(alpha.*(180/pi), beta.*(180/pi));
figure;
surf(A, B, max_ros_map);
xlabel('\alpha (rad)', 'FontSize', 16);
ylabel('\beta (rad)', 'FontSize', 16);
zlabel('Risk of Slip', 'FontSize', 16);
xlim([30, 60]);
ylim([0, 15]);
zlim([0.15, 0.45]);
view(-45, 45);
clim([0.17, 0.3]);
colorbar;
ax = gca;
ax.FontSize = 14;
title('Four-wheel Axis Symmetrical', 'FontSize', 16);
set(gcf, 'Position', [817, 612, 560, 350]); 

% Plots for mirror configuration
% [A, B] = meshgrid(alpha.*(180/pi), beta.*(180/pi));
% figure;
% subplot(2, 2, 1);
% surf(A, B, max_ros_map(:, :, 1));
% xlabel("\alpha (rad)");
% ylabel("\beta (rad)");
% zlabel("Risk of Slip");
% title("\gamma = 0^{\circ}");
% xlim([30, 80]); ylim([0, 90]); zlim([0, 1]);
% view(-215, 60);
% subplot(2, 2, 2);
% surf(A, B, max_ros_map(:, :, 2));
% xlabel("\alpha (rad)");
% ylabel("\beta (rad)");
% zlabel("Risk of Slip");
% title("\gamma = 45^{\circ}");
% xlim([30, 80]); ylim([0, 90]); zlim([0, 1]);
% view(-215, 60);
% subplot(2, 2, 3);
% surf(A, B, max_ros_map(:, :, 3));
% xlabel("\alpha (rad)");
% ylabel("\beta (rad)");
% zlabel("Risk of Slip");
% title("\gamma = 90^{\circ}");
% xlim([30, 80]); ylim([0, 90]); zlim([0, 1]);
% view(-215, 60);
% subplot(2, 2, 4);
% surf(A, B, max_ros_map(:, :, 4));
% xlabel("\alpha (rad)");
% ylabel("\beta (rad)");
% zlabel("Risk of Slip");
% title("\gamma = 135^{\circ}");
% xlim([30, 80]); ylim([0, 90]); zlim([0, 1]);
% view(-215, 60);

% Plot for gamma angle comparison
% alpha_idx = 1;
% beta_idx = 1;
% polarplot(gamma, reshape(max_ros_map(beta_idx, alpha_idx, :),1,[]));
% title("\alpha = 45^{\circ}, \beta = -10^{\circ}");


%% Show total running time
toc;


