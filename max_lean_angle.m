

    % Calculate roatation matrix
    phi = traj_sol(idx);
    [r_vec_world, R_world2wheel] = rotation_matrix(phi, params);
    
    % Calculaye stiffness matrix at body frame
    for i = 1:4
        stiffness_matrix_ball{i} = R_world2wheel{i} * stiffness_matirx_wheel_ball * R_world2wheel{i}';
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
        ft_world_frame{i} = R_world2wheel{i} * ft_wheel_frame{i};
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