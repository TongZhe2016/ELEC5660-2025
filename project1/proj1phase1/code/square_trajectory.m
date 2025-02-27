function s_des = square_trajectory(t, true_s)
    s_des = zeros(11,1);
% Given yaw, DO NOT CHANGE
    yaw_des = mod(0.2 * pi * t,2 * pi);
    dyaw_des = 0.2 * pi;
    
% TODO: Implement square trajectory here
    omega=25;
    
    

    s_des(1)=x_des; 
    s_des(2)=y_des; 
    s_des(3)=z_des; 
    s_des(4)=x_vdes; 
    s_des(5)=y_vdes; 
    s_des(6)=z_vdes;
    s_des(7)=x_ades; 
    s_des(8)=y_ades; 
    s_des(9)=z_ades;
    s_des(10)=yaw_des; 
    s_des(11)=dyaw_des; 
    
end
