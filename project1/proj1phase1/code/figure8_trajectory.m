% s_des(1:3) 目标位置
% s_des(4:6) 目标速度
% s_des(7:9) 目标加速度
% s_des(10) 目标偏航角（欧拉角）
% s_des(11) 目标偏航角速度（欧拉角）
function s_des = figure8_trajectory(t, true_s)
    s_des = zeros(11,1);
    % 给定偏航角参数（保持原有设置）
    yaw_des = mod(0.2 * pi * t, 2 * pi);  % 偏航角随时间线性变化
    dyaw_des = 0.2 * pi;                   % 恒定偏航角速度
    
    % ========== 8字轨迹参数设置 ==========
    T = 10;          % 轨迹周期（秒）
    A = 5;         % x方向振幅（米）
    B = 2;         % y方向振幅（米）
    omega = 2*pi/T;  % 基础角频率（rad/s）
    
    % ========== 轨迹生成核心计算 ==========
    % 位置计算（莱萨茹曲线参数方程）
    x_des = A * sin(omega*t);
    y_des = B * sin(2*omega*t);  % 2倍频形成8字
    
    % 速度计算（位置的一阶导数）
    x_vdes = A*omega * cos(omega*t);
    y_vdes = 2*B*omega * cos(2*omega*t);
    
    % 加速度计算（位置的二阶导数）
    x_ades = -A*omega^2 * sin(omega*t);
    y_ades = -4*B*omega^2 * sin(2*omega*t);
    
    % ========== z轴保持固定高度 ==========
    z_des = 1;        % 固定高度1米
    z_vdes = 0;       % 垂直速度为零
    z_ades = 0;       % 垂直加速度为零
    
    % ========== 填充输出向量 ==========
    s_des(1:3) = [x_des; y_des; z_des];         % 位置
    s_des(4:6) = [x_vdes; y_vdes; z_vdes];      % 速度
    s_des(7:9) = [x_ades; y_ades; z_ades];      % 加速度
    s_des(10) = yaw_des;                        % 偏航角
    s_des(11) = dyaw_des;                       % 偏航角速度
end