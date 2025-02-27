% s_des(1:3) 目标位置
% s_des(4:6) 目标速度
% s_des(7:9) 目标加速度
% s_des(10) 目标偏航角（欧拉角）
% s_des(11) 目标偏航角速度（欧拉角）
function s_des = square_trajectory(t, true_s)
    s_des = zeros(11,1);
    % 航点定义（包含起点和终点）
    waypoints = [0, 0, 0;
                 1, 2, 0;
                 2, 2, 2;
                 3, 0, 2;
                 4, 0, 0];
    
    % 轨迹参数
    T_total = 25;           % 总任务时间
    num_segments = size(waypoints,1)-1;  % 航段数量
    T_segment = T_total/num_segments;    % 单航段时间
    
    % 偏航角参数（保持原有设置）
    yaw_des = mod(0.2*pi*t, 2*pi);
    dyaw_des = 0.2*pi;

    % 轨迹计算
    if t >= T_total
        % 任务完成后保持最终状态
        final_wp = waypoints(end,:);
        s_des(1:3) = final_wp';
        s_des(4:9) = zeros(6,1);
    else
        % 确定当前航段
        segment = floor(t/T_segment) + 1;
        start_wp = waypoints(segment,:);
        end_wp = waypoints(segment+1,:);
        
        % 计算当前段内时间进度
        t_segment_start = (segment-1)*T_segment;
        delta_t = t - t_segment_start;
        ratio = delta_t/T_segment;
        
        % 线性插值计算
        pos = start_wp + (end_wp - start_wp)*ratio;
        vel = (end_wp - start_wp)/T_segment;
        
        % 填充状态量
        s_des(1:3) = pos';
        s_des(4:6) = vel';
        s_des(7:9) = zeros(3,1);  % 加速度为零
    end
    
    % 固定高度参数
    s_des(10) = yaw_des;
    s_des(11) = dyaw_des;
end