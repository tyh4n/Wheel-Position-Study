% Created by Tianyi Han, Jun.20 2023

function [max_ros] = ros_calculation(params)

    % Fetch data from dictionary
    l = params("l");
    r = params("r");
    g = params("g");
    m_body = params("m_body");
    m_ball = params("m_ball");
    k_wheel_ball = params("k_wheel_ball");
    f_ball_arrestor = params("f_ball_arrestor");

    % Load trajectory from the file
    load("data/traj_1.mat");

    % Reset max risk of slip
    max_ros = 0;

    % Stiffness matrix at contact frame
    stiffness_matirx_wheel_ball = [[0, 0, 0           ];
                                   [0, 0, 0           ];
                                   [0, 0, k_wheel_ball]];
    stiffness_matrix_ball  = {[], [], [], []};

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
    f_body_ball = f_ball_arrestor_vec + [zeros([1, num_step]); zeros([1, num_step]); - ones([1, num_step]) * m_body * g] - m_body * accel_body;
    f_ground_ball_normal =  - f_body_ball + [zeros([1, num_step]); zeros([1, num_step]); ones([1, num_step]) * m_ball * g];

    % Init storage variable
    ft_sol_store = zeros([4, num_step]);
    fn_sol_store = zeros([4, num_step]);
    ros_sol_store = zeros([4, num_step]);

    % Loop through time steps
    for idx = 1:num_step
        % Calculate roatation matrix
        phi = traj_sol(idx);
        [r_vec_world, R_wheel2world] = rotation_matrix(phi, params);

        % Calculaye stiffness matrix at body frame
        for i = 1:4
            stiffness_matrix_ball{i} = R_wheel2world{i} * stiffness_matirx_wheel_ball * R_wheel2world{i}';
        end

        % Calculate simplified A matrix
        A_matrix = zeros([3, 3]);
        for i = 1:4
            A_matrix = A_matrix + stiffness_matrix_ball{i};
        end

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
        if params("torque_distribution") == 0 
            eqn3 = ft1 == - ft3;
        elseif  params("torque_distribution") == 1 
            alpha = params("alpha");
            fn_zero = 1/4 * m_body * g / cos(alpha);
            eqn3 = ft1 * (fn_zero + f_body_ball(1, idx)/(2 * sin(alpha))) == - ft3 * (fn_zero - f_body_ball(1, idx)/(2 * sin(alpha)));
        end
        
        % Solve multi-variables function
        eqns = [eqn1; eqn2; eqn3];
        S = solve(eqns,[ft1 ft2 ft3 ft4 delta_ball_x delta_ball_y delta_ball_z]);

        % Compare ROS solution
        ft_sol = double(subs(ft, S));
        fn1_sol = double(subs(stiffness_matrix_ball{1} * delta_ball, S));
        fn2_sol = double(subs(stiffness_matrix_ball{2} * delta_ball, S));
        fn3_sol = double(subs(stiffness_matrix_ball{3} * delta_ball, S));
        fn4_sol = double(subs(stiffness_matrix_ball{4} * delta_ball, S));
        fn_sol = [norm(fn1_sol) norm(fn2_sol) norm(fn3_sol) norm(fn4_sol)];
        ros_sol = abs(ft_sol ./ fn_sol);
        max_ros = max(max_ros, max(ros_sol));

        % Display solution
        % disp("-------------------------------------");
        % disp("Step: ");
        % disp(idx);
        % disp("F_traction: ");
        % disp(ft_sol);
        % disp("F_normal: ");
        % disp(fn_sol);
        % disp("Risk of slip: ");
        % disp(ros_sol);

        % Store solution
        ft_sol_store(:, idx) = ft_sol';
        fn_sol_store(:, idx) = fn_sol';
        ros_sol_store(:, idx) = ros_sol';
    end

    % Plot result
    % f = figure;
    % subplot(3, 1, 1);
    % hold on;
    % plot(time, ft_sol_store(1, :));
    % plot(time, ft_sol_store(2, :));
    % plot(time, ft_sol_store(3, :));
    % plot(time, ft_sol_store(4, :));
    % xlabel('Time (s)');
    % ylabel('F_{traction} (N)');
    % title('Traction Force');
    % legend('Wheel 1', 'Wheel 2', 'Wheel 3', 'Wheel 4');
    % subplot(3, 1, 2);
    % hold on;
    % plot(time, fn_sol_store(1, :));
    % plot(time, fn_sol_store(2, :));
    % plot(time, fn_sol_store(3, :));
    % plot(time, fn_sol_store(4, :));
    % xlabel('Time (s)');
    % ylabel('F_{normal} (N)');
    % title('Normal Force');
    % legend('Wheel 1', 'Wheel 2', 'Wheel 3', 'Wheel 4');
    % subplot(3, 1, 3);
    % hold on;
    % plot(time, ros_sol_store(1, :));
    % plot(time, ros_sol_store(2, :));
    % plot(time, ros_sol_store(3, :));
    % plot(time, ros_sol_store(4, :));
    % xlabel('Time (s)');
    % ylabel('Risk of Slip');
    % title('Risk of Slip');
    % legend('Wheel 1', 'Wheel 2', 'Wheel 3', 'Wheel 4');
    % sg_title = "Ft, Fn, and RoS at \alpha = " + num2str(params("alpha")/(pi/180)) + ...
    %     ", \beta = " + num2str(params("beta")/(pi/180)) + ", \gamma = " + num2str(params("gamma")/(pi/180));
    % sgtitle(sg_title);
    % f.Position = [100 100 500 800];
end



