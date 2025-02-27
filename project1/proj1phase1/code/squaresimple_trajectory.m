% s_des(1:3) 目标位置
% s_des(4:6) 目标速度
% s_des(7:9) 目标加速度
% s_des(10) 目标偏航角（欧拉角）
% s_des(11) 目标偏航角速度（欧拉角）
function s_des = squaresimple_trajectory(t, true_s)
    s_des = zeros(11,1);
    % Given yaw, DO NOT CHANGE
    yaw_des = mod(0.2 * pi * t, 2 * pi);
    dyaw_des = 0.2 * pi;
    
    % Square trajectory parameters
    T_segment = 5;      % Time per segment (each side of the square)
    side_length = 3;     % Length of each side
    vel = side_length / T_segment; % Constant velocity per segment
    T_total = 4 * T_segment; % Total time for the square trajectory
    t_mod = mod(t, T_total); % Modulo time for cyclic trajectory
    
    % Initialize position and velocity variables
    x_des = 0;
    y_des = 0;
    x_vdes = 0;
    y_vdes = 0;
    
    % Determine current segment and compute desired states
    if t_mod < T_segment
        % Segment 1: Move along x-axis from (0,0) to (3,0)
        x_des = vel * t_mod;
        y_des = 0;
        x_vdes = vel;
        y_vdes = 0;
    elseif t_mod < 2*T_segment
        % Segment 2: Move along y-axis from (3,0) to (3,3)
        delta_t = t_mod - T_segment;
        x_des = side_length;
        y_des = vel * delta_t;
        x_vdes = 0;
        y_vdes = vel;
    elseif t_mod < 3*T_segment
        % Segment 3: Move along x-axis back from (3,3) to (0,3)
        delta_t = t_mod - 2*T_segment;
        x_des = side_length - vel * delta_t;
        y_des = side_length;
        x_vdes = -vel;
        y_vdes = 0;
    else
        % Segment 4: Move along y-axis back from (0,3) to (0,0)
        delta_t = t_mod - 3*T_segment;
        x_des = 0;
        y_des = side_length - vel * delta_t;
        x_vdes = 0;
        y_vdes = -vel;
    end
    
    % Z-axis remains constant at 3 meter with zero velocity and acceleration
    z_des = 3;
    z_vdes = 0;
    z_ades = 0;
    
    % Acceleration components are zero (assumes instantaneous velocity changes)
    x_ades = 0;
    y_ades = 0;
    
    % Populate the desired state vector
    s_des(1) = x_des; 
    s_des(2) = y_des; 
    s_des(3) = z_des; 
    s_des(4) = x_vdes; 
    s_des(5) = y_vdes; 
    s_des(6) = z_vdes;
    s_des(7) = x_ades; 
    s_des(8) = y_ades; 
    s_des(9) = z_ades;
    s_des(10) = yaw_des; 
    s_des(11) = dyaw_des; 
end
