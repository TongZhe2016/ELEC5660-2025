function [F, M] = controller(t, s, s_des)

global params

m = params.mass;
g = params.grav;
I = params.I;

% s(1:3) current position
% s(4:6) current velocity
% s(7:10) current attitude quaternion
% s(11:13) current body angular velocity

% s_des(1:3) desire position
% s_des(4:6) desire velocity
% s_des(7:9) desire acceleration
% s_des(10) desire yaw
% s_des(11) desire yaw rate

F = 1.0; M = [0.0, 0.0, 0.0]'; % You should calculate the output F (Thrust) and M (Moment)

end
