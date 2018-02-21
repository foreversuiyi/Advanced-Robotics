clc
clear
close all
%% Initial parameters
q(1,:) = [-pi/4, pi/4, 0]';
q(2,:) = [-pi/4, 0, pi/4]';
Force = [4, 1, 3]';
%% Display
for k = 1:2
    [link_x, link_y, link_z, Jacobian] = problem1_fkine(q(k,:));
    Torque = Jacobian'*Force;
    disp('Torque = '); disp(Torque);
    disp('End')
end