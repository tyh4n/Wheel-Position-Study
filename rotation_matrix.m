% Created by Tianyi Han, Jun.19 2023

function [r_vec_world, R_wheel2world] = rotation_matrix(phi, params)

    % Fetch data from dictionary
    alpha = params("alpha");
    beta = params("beta");
    gamma = params("gamma");
    r = params("r");

    % Create empty cell for rotation matrix
    R_wheel2body = {[], [], []};
    R_wheel2world = {[], [], []};

    % Calculate rotation matrix
    for i = 1:4
        if ((params("symmetric_type") == 1) && (mod(i, 2) == 1))
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

    % Calculate r vector
    r_vec_body = {[], [], []};
    r_vec_world = {[], [], []};
    for i = 1:4
        r_vec_body{i} = [r * sin(alpha) * cos(gamma + (i - 1) * pi/2); r * sin(alpha) * sin(gamma + (i - 1) * pi/2); r * cos(alpha)];
        r_vec_world{i} = R_body2world * r_vec_body{i};
    end
end
