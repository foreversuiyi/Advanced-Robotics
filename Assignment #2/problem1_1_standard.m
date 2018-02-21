clc
clear
close all
%% Initial parameters
q(1,:) = [-pi/4, pi/4, 0]';
q(2,:) = [-pi/4, 0, pi/4]';
q_dot = [pi, pi/2, 0]';
%% Display
for k = 1:2
    [link_x, link_y, link_z, Jacobian] = problem1_fkine(q(k,:));
    vel = Jacobian * q_dot;
    disp('vel = '); disp(vel')
    Manip = Jacobian * Jacobian';
    disp('Manip = '); disp(det(Manip));
    disp('End')
end