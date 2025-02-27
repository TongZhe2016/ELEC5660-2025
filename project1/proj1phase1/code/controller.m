% s(1:3) 实际位置
% s(4:6) 实际速度
% s(7:10) 实际姿态四元数
% s(11:13) 实际角速度（欧拉角）
    
% s_des(1:3) 目标位置
% s_des(4:6) 目标速度
% s_des(7:9) 目标加速度
% s_des(10) 目标偏航角（欧拉角）
% s_des(11) 目标偏航角速度（欧拉角）
function [F, M] = controller(t, s, s_des)

    global params
    m = params.mass;
    g = params.grav;
    I = params.I;
    
    F = 1.0; M = [0.0, 0.0, 0.0]'; % You should calculate the output F and M
    %% 获得位置误差和速度误差
    position_error = s_des(1:3) - s(1:3);
    velocity_error = s_des(4:6) - s(4:6);
    
    global 	last_t position_error_int
    if isempty(last_t)
        last_t = t;
    end
    dt = t - last_t;
    last_t = t;
    if isempty(position_error_int)
        position_error_int = [0.0, 0.0, 0.0]';
    end
    position_error_int =  dt * position_error +position_error_int;


    %% 位置环，输出总竖直推力
    Kp_position = [5.0, 9.0, 18.5]'; % xyz position proportional gain
    Ki_position = [0.00001, 0.00001, 0.00001]'; % xyz position integral gain
    Kd_position = [7.0, 7.0, 22.0]'; % xyz position derivative(velocity) gain
    
    % PID 控制器，在目标加速度的基础上加上控制的xyz方向的加速度
    acc_des = s_des(7:9) + Kp_position.*position_error + Ki_position.*position_error_int + Kd_position.*velocity_error;
    F = m*(g+acc_des(3));


    %% 姿态环，使用位置环的输出作为目标加速度
    % 获得实际姿态角度和角速度
    q = s(7:10);
    [phi, theta, psi] = RotToRPY_ZXY(quaternion_to_R(q));
    omega = [s(11);s(12);s(13)];
    
    % desired angles: phi, theta, psi
    phi_des = 1/g*(acc_des(1)*sin(psi)-acc_des(2)*cos(psi));
    theta_des = 1/g*(acc_des(1)*cos(psi)+acc_des(2)*sin(psi));
    psi_des = s_des(10);
    
    % desired angular velocities: phi_v, theta_v, psi_v
    global phi_last_des theta_last_des
    phi_v_des = EulerAngleClamp(phi_des - phi_last_des) / dt; % 使用Garyandtang的EulerAngleClamp函数[1]将角度限制在[-pi, pi]之间
    theta_v_des = EulerAngleClamp(theta_des - theta_last_des) / dt; % 使用Garyandtang的EulerAngleClamp函数[1]将角度限制在[-pi, pi]之间
    phi_last_des = phi_des;
    theta_last_des = theta_des;
    psi_v_des = s_des(11);
    
    % 调姿态环用，将目标姿态、角速度设置为0
    % phi_des = 0.0;
    % theta_des = 0.0;
    % psi_des = 0.0;
    % phi_v_des = 0.0;
    % theta_v_des = 0.0;
    % psi_v_des = 0.0;

    % parameters
    Kp_phi = 312.5;
    Kp_theta = 125;
    Kp_psi = 250;
    Kd_phi = 80;
    Kd_theta = 80;
    Kd_psi = 62.5;

    phi_acc_psi = Kp_phi * EulerAngleClamp(phi_des-phi) + Kd_phi * (phi_v_des - omega(1));
    theta_acc_psi = Kp_theta * EulerAngleClamp(theta_des-theta) + Kd_theta * (theta_v_des - omega(2));
    psi_acc_psi = Kp_psi * EulerAngleClamp(psi_des-psi) + Kd_psi * (psi_v_des - omega(3));
    omega_v_des =[phi_acc_psi; theta_acc_psi; psi_acc_psi];
    M=I*omega_v_des+cross(omega,I*omega);
end


%% [1]Function from https://github.com/Garyandtang/ELEC5660-2021 By Garyandtang, 2021
% This function is used to clamp the angle to [-pi, pi]
% works well when the angle change suddenly, for example, from pi to -pi, which are actually the same angle
% However, if we do not implement this function, the drone will try to rotate from pi to -pi, which is not what we want
function [result] =  EulerAngleClamp(angle)
    result = atan2(sin(angle), cos(angle));
end