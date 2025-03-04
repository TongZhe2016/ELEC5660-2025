%% 实现无人机的轨迹规划，Optimization-based trajectory generation. 目标是最小化snap
% 输入：当前时间t，路径path
% 输出：无人机的目标状态s_des的定义如下：
% s_des(1:3) 目标位置
% s_des(4:6) 目标速度
% s_des(7:9) 目标加速度
% s_des(10) 目标偏航角（欧拉角）
% s_des(11) 目标偏航角速度（欧拉角）

function s_des = trajectory_generator(t, path)
global time_tol; % 在run_trajectory.m中定义
persistent polynomial_indexes
persistent des_positions
persistent time_points
persistent time_intervals

if nargin > 1
    %% 生成轨迹
    % 在test_trajectory.m中，如果想要生成轨迹，在调用此函数时会输入[]和path，
    % 而在run_trajectory.m中，只会输入t。因此可以通过参数数量（nargin）来判断是否需要生成轨迹
    
    %% 分割成不同的段，并计算每段的时间
    waypoint_diff = path(2:end,:)-path(1:end-1,:); % 每段的位移
    path_lengths = sqrt(sum(waypoint_diff.^2,2)); % 每段的长度
    total_length = sum(path_lengths); % 总长度
    % 用一个比较直接的方法，即分配的时间长度与段的长度成正比
    time_intervals = time_tol.*path_lengths./total_length; % 每段的时间长度
    time_points = cumsum(time_intervals); % 每段的时间点
    time_points = [0;time_points]; % 加上初始时间点0

    %% cost function
    N = 8; % minimum snap
    M = length(path)-1;
    Q_all = zeros(N*M,N*M); %block diagonal matrix
    polynomial_indexes = zeros(N*M,size(path,2));
    for k = 1:M
        for i = 4:8
            for j = 4:8
                Q_all(N*(k-1)+i,N*(k-1)+j) = i*(i-1)*(i-2)*(i-3)*j*(j-1)*(j-2)*(j-3)/(i+j-7)*time_intervals(k)^(i+j-7);
            end
        end
    end
    %% Constraints
    % model waypoints constraints f_j^{(0)}(T_j)=x_j^{(0)}
    d_positions = kron(path, ones(2,1));
    d_positions = d_positions(2:end-1,:);
    A_positions = zeros(size(d_positions,1),M*N);

    for j = 1 : M
        A_positions((1+2*(j-1)):(2+2*(j-1)),(8*j-7):(8*j))= ...
            [1, 0, 0, 0, 0, 0, 0, 0; ...
            1, time_intervals(j)^1,time_intervals(j)^2, time_intervals(j)^3,time_intervals(j)^4,time_intervals(j)^5,time_intervals(j)^6,time_intervals(j)^7];
    end

    % continuity cosntraints
    % position + velocity + acceleration + ya
    KK = 4; % hyper parameter on no. of constraints
    A_continuity = zeros(KK*(M-1),M*N);
    d_continuity = zeros(KK*(M-1), size(path,2));
    for j = 1 : M-1
        A_continuity((1+KK*(j-1)):(KK+KK*(j-1)),(1+8*(j-1)):(16+8*(j-1)))= ...
            [1, time_intervals(j)^1,time_intervals(j)^2, time_intervals(j)^3,time_intervals(j)^4,time_intervals(j)^5,time_intervals(j)^6,time_intervals(j)^7,-1,0,0,0,0,0,0,0; ...
            0, time_intervals(j)^0,2*time_intervals(j)^1, 3*time_intervals(j)^2,4*time_intervals(j)^3,5*time_intervals(j)^4,6*time_intervals(j)^5,7*time_intervals(j)^6,0,-1,0,0,0,0,0,0; ...
            0, 0*time_intervals(j)^0,2*time_intervals(j)^0, 6*time_intervals(j)^1,12*time_intervals(j)^2,20*time_intervals(j)^3,30*time_intervals(j)^4,42*time_intervals(j)^5,0,0,-2,0,0,0,0,0;...
            0, 0*time_intervals(j)^0,0*time_intervals(j)^0, 6*time_intervals(j)^0,24*time_intervals(j)^1,60*time_intervals(j)^2,120*time_intervals(j)^3,210*time_intervals(j)^4,0,0,0,-6,0,0,0,0;];
    end
    A_eq = [A_positions;A_continuity];
    d_eq = [d_positions;d_continuity];
    f = zeros(M*N,1);
    polynomial_indexes(:,1) = quadprog(Q_all,f,[],[],A_eq,d_eq(:,1));
    polynomial_indexes(:,2) = quadprog(Q_all,f,[],[],A_eq,d_eq(:,2));
    polynomial_indexes(:,3) = quadprog(Q_all,f,[],[],A_eq,d_eq(:,3));

else
    %% 当只有一个输入参数时，即t，输出计算好的轨迹s_des
    s_des = zeros(13,1);
    if t > time_points(end)
        s_des(1:3) = des_positions;
        return
    end
    for i = 1 : size(time_points,1)
        if t >= time_points(i) && t <time_points(i+1)
            time = time_points(i);
            index = i;
            break
        end
    end
    T = t - time;
    p = polynomial_indexes((1+8*(index-1)):(8+8*(index-1)),:);
    s_des(1:3) = [1,T,T^2,T^3,T^4,T^5,T^6,T^7]*p;
    des_positions = s_des(1:3);
    s_des(4:6) = [0,1*T^0,2*T^1,3*T^2,4*T^3,5*T^4,6*T^5,7*T^6]*p;
    s_des(7:9) = [0,0*T^0,2*T^0,6*T^1,12*T^2,20*T^3,30*T^4,42*T^5]*p;
end

