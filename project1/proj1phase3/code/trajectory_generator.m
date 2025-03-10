function s_des = trajectory_generator(t, path, h, map)

if nargin > 1 % pre-process can be done here (given waypoints). Pre-define the entire trajectory.

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

else % output desired trajectory here (given time)
    
end

end


