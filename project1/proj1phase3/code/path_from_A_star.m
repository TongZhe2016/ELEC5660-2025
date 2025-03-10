function Optimal_path = path_from_A_star(map)
%% 获得地图大小并初始化。
% 如果map里面所有的点都在一个平面上，那么就是2D的
% 每个点的定义：-1表示障碍物，0表示目标，1表示起点，2表示空闲

% 判断是否是3D地图，防止出现2D地图直接从上面飞过去的情况
is3D = 0;
map_size = size(map,1);
for i = 1 : map_size
    if map(i,3) ~= map(1,3) % 如果有一个点的z坐标和第一个点的z坐标不一样，那么就是3D的
        is3D = 1;
        break
    end
end
% 基于map里面各项的位置，初始化地图。
if is3D == 0
    x_Max = max(map(:,1))+1;
    y_Max = max(map(:,2))+1;
    % 限制无人机的高度只能在同一个平面上
    z_Max = max(map(:,3));
    z_Min = min(map(:,3));
else
    % 地图大小比最大的坐标值大1，供无人机活动
    x_Max = max(map(:,1))+1;
    y_Max = max(map(:,2))+1;
    z_Max = max(map(:,3))+1;
    z_Min = min(map(:,3))-1;
end
% 将所有的点都初始化为2（free）
MAP = ones(x_Max , y_Max , z_Max);
MAP = 2 .* MAP;
% 初始化起点
Start = map(1,:);
MAP(Start(1),Start(2),Start(3)) = 1;
% 初始化目标点
Des = map(map_size,:);
MAP(Des(1),Des(2),Des(3)) = 0;
% 初始化障碍点
for i = 2: (map_size-1)
    MAP(map(i,1),map(i,2),map(i,3)) = -1;
end

%% 初始化A*算法的数据
% 初始化g。对于未发现的点，g是无穷大
g = inf(x_Max,y_Max,z_Max);
% 记录哪些点已经被扩展了。已扩展：1，未扩展：0
expanded = zeros(x_Max,y_Max,z_Max);
% 记录每个点的父节点
parents = zeros(x_Max,y_Max,z_Max, 3); % 3是因为每个点有三个坐标
% Container storing nodes to be expanded, along with the f score (f=g+h)
% Each node's (x,y,z) coordinate and its f score is stored in a row
queue = [Start(1),Start(2),Start(3),0]; % 数据格式：[x1, y1, z1, f1; x2, y2, z2, f2; ...; xn, yn, zn, fn]
% 最佳路径
Optimal_path = [];

%% A*算法
while 1
    % 当队列为空，且没有找到路径时，说明没有可行的路径
    if size(queue,1) == 0
        error('No path found');
    end
    f = queue(:,4); % f = g + h
    [index,~] = find(f == min(f)); % 找到f最小的点，index是这些点的索引
    least_f_xyz = queue(index(1),1:3); % 找到f最小的点的坐标
    g_n = queue(index(1),4); % 找到f最小的点的g值
    queue(index(1),:) = []; % 移除f最小的点
    expanded(least_f_xyz(1),least_f_xyz(2),least_f_xyz(3)) = 1; % 标记这个点已经被扩展了
    %% 如果找到了目标点，就回溯找到路径
    if least_f_xyz == Des
        while 1
            Optimal_path = [least_f_xyz;Optimal_path];
            if least_f_xyz == Start % 如果回溯到了起点，就结束
                disp('Path found!');
                break
            end
            x = least_f_xyz(1);
            y = least_f_xyz(2);
            z = least_f_xyz(3);
            least_f_xyz = [parents(x,y,z,1),parents(x,y,z,2),parents(x,y,z,3)]; % 回溯一个点
        end
        break
    end

    % 如果没连接到目标点，扩展当前点附近的点继续找

    %% 获得当前点的邻居（不包含超出地图范围的点和障碍物）
    % 如果是3D地图，邻居有6个方向，如果是2D地图，邻居有4个方向
    if is3D == 1
        neighbors = [least_f_xyz(1)+1 least_f_xyz(2) least_f_xyz(3) ; ...
            least_f_xyz(1)-1 least_f_xyz(2) least_f_xyz(3) ; ...
            least_f_xyz(1) least_f_xyz(2)+1 least_f_xyz(3) ; ...
            least_f_xyz(1) least_f_xyz(2)-1 least_f_xyz(3) ; ...
            least_f_xyz(1) least_f_xyz(2) least_f_xyz(3)+1 ; ...
            least_f_xyz(1) least_f_xyz(2) least_f_xyz(3)-1 ];
    else
        neighbors = [least_f_xyz(1)+1 least_f_xyz(2) least_f_xyz(3) ; ...
            least_f_xyz(1)-1 least_f_xyz(2) least_f_xyz(3) ; ...
            least_f_xyz(1) least_f_xyz(2)+1 least_f_xyz(3) ; ...
            least_f_xyz(1) least_f_xyz(2)-1 least_f_xyz(3) ];
    end
    % 去掉超出地图范围的点和障碍物
    invalid_index = [];
    for i = 1 : size(neighbors,1)
        if min(neighbors(i,:)) < 1 || ...
                max(neighbors(i,1)) > x_Max || ...
                max(neighbors(i,2)) > y_Max || ...
                max(neighbors(i,3)) > z_Max || ...
                min(neighbors(i,3)) < z_Min || ...
                MAP(neighbors(i,1),neighbors(i,2),neighbors(i,3)) == -1 % 如果是障碍物
            invalid_index = [invalid_index;i]; % 记录这个点的索引
        end
    end
    invalid_index = sort(invalid_index, 'descend'); % 从后往前删除，避免删除后索引变化
    for i = 1 : size(invalid_index,1)
        neighbors(invalid_index(i),:)=[]; % 去掉超出地图范围的点和障碍物
    end

    %% 扩展当前点的邻居
    for i = 1 : size(neighbors,1)
        x = neighbors(i,1);y = neighbors(i,2);z = neighbors(i,3);
        if expanded(x,y,z) == 0
            g_m = g_n + 1 + h(neighbors(i),Des);
            if g(x,y,z) == inf
                queue = [[neighbors(i,:),g_m];queue];
            end
            if g(x,y,z) > g_n + 1
                parents(x,y,z,:) = least_f_xyz;
                g(x,y,z) = g_m;
            end
        end
    end
end
end

%% heuristic function A*
function h_value = h(curr_point, tar_point)
h_value = sum(abs(tar_point-curr_point)); % 这里采用曼哈顿距离，因为无人机不能斜着飞
end