% Created by Tianyi Han, Jun.12 2023

%% Define body_ball force in wheel frame
ft = 100; % [N]
fn = 0; % [N]
f_wheel_frame = [ft, 0, fn]';

%% Define robot configuration
alpha = 45./(180/pi);    % [rad]
beta = -10./(180/pi);    % [rad]
gamma = pi/4;      % [rad]
r = 125;        % [mm]
phi = 0;     % [rad]

%% Define rotation matrix
R_wheel2body = {[], [], []};
R_wheel2world = {[], [], []};
for i = 1:4
    if (mod(i, 2) == 1)
        R1 = [[cos(beta),  -sin(beta),    0];
              [sin(beta),  cos(beta),     0];
              [0,           0,              1]];
    else
        R1 = [[cos(-beta),  -sin(-beta),    0];
              [sin(-beta),  cos(-beta),     0];
              [0,           0,              1]];
    end
    R2 = [[1,   0,                  0             ];
          [0,   cos(alpha - pi),    -sin(alpha - pi)];
          [0,   sin(alpha - pi),    cos(alpha - pi)]];
    R3 = [[cos(gamma + i * pi/2),   -sin(gamma + i * pi/2), 0];
          [sin(gamma + i * pi/2),   cos(gamma + i * pi/2),  0];
          [0,                       0,                      1]];
    R_wheel2body{i} = R3 * R2 * R1;
end
R_body2world = [[cos(phi),     0,      sin(phi)];
                [0,            1,      0,     ];
                [-sin(phi),    0,      cos(phi)]];
for i = 1:4
    R_wheel2world{i} = R_wheel2body{i} * R_body2world;
end

%% Calculate force vector
f_world_frame = {[], [], []};
for i = 1:4
    f_world_frame{i} = R_wheel2world{i} * f_wheel_frame;
end
r_vec_body = {[], [], []};
r_vec_world = {[], [], []};
for i = 1:4
    r_vec_body{i} = [r * sin(alpha) * cos(gamma + (i - 1) * pi/2); r * sin(alpha) * sin(gamma + (i - 1) * pi/2); r * cos(alpha)];
    r_vec_world{i} = R_body2world * r_vec_body{i};
end

%% Plot Force
[X,Y,Z] = sphere(100);
s = surf(X * r, Y * r, Z * r);
s.EdgeColor = 'none';
s.FaceColor = [.8, .8, .8];
s.FaceAlpha = 0.5;
hold on;

% Plot the vector
for i = 1:4
    quiver3(0, 0, 0, r_vec_world{i}(1), r_vec_world{i}(2), r_vec_world{i}(3), 'b', 'LineWidth', 2);
    quiver3(r_vec_world{i}(1), r_vec_world{i}(2), r_vec_world{i}(3), f_world_frame{i}(1), f_world_frame{i}(2), f_world_frame{i}(3), 'r', 'LineWidth', 2);
end

% Set the axis labels
xlabel('X (mm, N)');
ylabel('Y (mm, N)');
zlabel('Z (mm, N)');

% Show the grid
grid on;
axis equal;

%% Save the rotation matirx
% save('rotation_matrix_1.mat', 'r_vec_world', 'R_wheel2world');