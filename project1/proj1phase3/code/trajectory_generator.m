%% 实现无人机的轨迹规划，Optimization-based trajectory generation. 目标是最小化snap
% 输入：当前时间t，路径path
% 输出：无人机的目标状态s_des的定义如下：
% s_des(1:3) 目标位置
% s_des(4:6) 目标速度
% s_des(7:9) 目标加速度
% s_des(10) 目标偏航角（欧拉角）
% s_des(11) 目标偏航角速度（欧拉角）

function s_des = trajectory_generator(t, path, h, map)
time_tol = 25;
persistent polynomial_indexes
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
    time_points = cumsum(time_intervals); % 每段的时间点，从零开始逐渐累加
    time_points = [0;time_points]; % 加上初始时间点0

    %% cost function
    N = 7 + 1; % 4即minimum snap，算上常数项总共八项，七次多项式
    M = length(path)-1; % 轨迹被划分为M段
    Q_all = zeros(N*M,N*M); % 生成的多项式轨迹系数
    polynomial_indexes = zeros(N*M,size(path,2)); % size(path,2) = 3，即xyz三维
    for k = 0:(M-1) % for each segment
        for i = 4:8
            for j = 4:8
                Q_all(N*k+i,N*k+j) = i*(i-1)*(i-2)*(i-3)*j*(j-1)*(j-2)*(j-3)/(i+j-7)*time_intervals(k+1)^(i+j-7);
            end
        end
    end
    %% Constraints
    b_pos_constrain = kron(path, ones(2,1)); % 位置约束
    b_pos_constrain = b_pos_constrain(2:end-1,:); % 去掉起点和终点
    A_pos_constrain = zeros(size(b_pos_constrain,1),M*N); % 位置约束矩阵

    for i = 1 : M
        A_pos_constrain((1+2*(i-1)):(2+2*(i-1)),(8*i-7):(8*i))= ...
            [1, 0, 0, 0, 0, 0, 0, 0; ...
            1, time_intervals(i)^1,time_intervals(i)^2, time_intervals(i)^3,time_intervals(i)^4,time_intervals(i)^5,time_intervals(i)^6,time_intervals(i)^7];
    end

    % 位置、速度、加速度、jerk的连续性约束
    A_continuity = zeros(4*(M-1),M*N);
    b_continuity = zeros(4*(M-1), size(path,2));
    for j = 1 : M-1
        A_continuity((1+4*(j-1)):(4+4*(j-1)),(1+8*(j-1)):(16+8*(j-1)))= ...
            [1, time_intervals(j)^1,time_intervals(j)^2, time_intervals(j)^3,time_intervals(j)^4,time_intervals(j)^5,time_intervals(j)^6,time_intervals(j)^7,-1,0,0,0,0,0,0,0; ...
            0, time_intervals(j)^0,2*time_intervals(j)^1, 3*time_intervals(j)^2,4*time_intervals(j)^3,5*time_intervals(j)^4,6*time_intervals(j)^5,7*time_intervals(j)^6,0,-1,0,0,0,0,0,0; ...
            0, 0*time_intervals(j)^0,2*time_intervals(j)^0, 6*time_intervals(j)^1,12*time_intervals(j)^2,20*time_intervals(j)^3,30*time_intervals(j)^4,42*time_intervals(j)^5,0,0,-2,0,0,0,0,0;...
            0, 0*time_intervals(j)^0,0*time_intervals(j)^0, 6*time_intervals(j)^0,24*time_intervals(j)^1,60*time_intervals(j)^2,120*time_intervals(j)^3,210*time_intervals(j)^4,0,0,0,-6,0,0,0,0;];
    end
    Aeq = [A_pos_constrain;A_continuity];
    beq = [b_pos_constrain;b_continuity];
    f = zeros(M*N,1); % 一次项系数向量
    %% 利用MATLAB的quadprog函数求解
    % MATLAB的quadprog函数用于求解二次规划（Quadratic Programming, QP）问题，其核心是优化带有
    % 线性约束的二次目标函数。在无人机轨迹生成场景中，它被用于求解满足连续性约束的最小Snap轨迹多项
    % 式系数。
    % quadprog(H, f, A, b, Aeq, beq, lb, ub, x0, options)
    % H: 二次项系数矩阵（必须对称）
    % f: 一次项系数向量
    % A, b: 不等式约束矩阵和向量（A*x ≤ b）
    % Aeq, beq: 等式约束矩阵和向量（Aeq*x = beq）
    % lb, ub: 变量下界和上界（lb ≤ x ≤ ub）
    % x0: 初始猜测解（可选）
    % options: 优化选项（如算法选择、容差等）

    polynomial_indexes(:,1) = quadprog(Q_all,f,[],[],Aeq,beq(:,1)); % x方向
    polynomial_indexes(:,2) = quadprog(Q_all,f,[],[],Aeq,beq(:,2)); % y方向
    polynomial_indexes(:,3) = quadprog(Q_all,f,[],[],Aeq,beq(:,3)); % z方向

    % visualize the 2D grid map
    subplot(h);
    % start point
    plot3(map(1, 1)-0.5, map(1, 2)-0.5, map(1, 3)-0.5, 'k.');
    hold on;
    % obstacles
    for obs_cnt = 2: size(map, 1) - 1
        plot3([map(obs_cnt, 1)-0.2 map(obs_cnt, 1)-0.8], [map(obs_cnt, 2)-0.2 map(obs_cnt, 2)-0.8], [map(obs_cnt, 3) map(obs_cnt, 3)], 'k-');
        hold on;
        plot3([map(obs_cnt, 1)-0.2 map(obs_cnt, 1)-0.8], [map(obs_cnt, 2)-0.8 map(obs_cnt, 2)-0.2], [map(obs_cnt, 3) map(obs_cnt, 3)], 'k-');
        hold on;
        ox = map(obs_cnt ,1) - 0.9;
        oy = map(obs_cnt ,2) - 0.9;
        oz = map(obs_cnt ,3) - 0.9;
        plotcube([0.8,0.8,0.8], [ox,oy,oz],1,[0.7,0.7,0.7]);
        grid minor
        set(gca,'xtick',[-100:1:100])
        set(gca,'ytick',[-100:1:100])
        grid off;
        grid on;
        axis equal;
        axis ([-1 6 -1 10 0 4]);
        hold on;
    end
    % target point
    plot3(map(obs_cnt+1, 1)-0.5, map(obs_cnt+1, 2)-0.5, map(obs_cnt+1, 3)-0.5, 'r*');
    hold on;

else
    %% 当只有一个输入参数时，即t，输出计算好的轨迹s_des
    s_des = zeros(13,1); % 初始化
    for i = 1 : size(time_points,1) % 找到当前时间点t所在的段index
        % 特殊情况处理：如果t在倒数第二段的时间点之后，即t>=time_points(end-1)，则取最后一段
        if t >= time_points(end-1)
            time = time_points(end-1);
            interval = size(time_points,1)-1;
            break
        end
        if t >= time_points(i) && t <time_points(i+1)
            time = time_points(i);
            interval = i;
            break
        end
    end
    T = t - time;
    p = polynomial_indexes((1+8*(interval-1)):(8+8*(interval-1)),:);
    s_des(1:3) = [1,T,T^2,T^3,T^4,T^5,T^6,T^7]*p;
    s_des(4:6) = [0,1*T^0,2*T^1,3*T^2,4*T^3,5*T^4,6*T^5,7*T^6]*p;
    s_des(7:9) = [0,0*T^0,2*T^0,6*T^1,12*T^2,20*T^3,30*T^4,42*T^5]*p;
end

end


