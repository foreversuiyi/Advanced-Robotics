clc; clear; close all
%% Initial parameters
k = 1:100;
max_f = 0;
max_theta = 0;
for s = 0:1
    theta = -((pi/2 * s /100));
    sim_ang = [-pi/4, theta, -2*theta]';
    [link_x,link_y,link_z,Jacobian] = problem1_fkine(sim_ang);
    F_Manip = Jacobian'* Jacobian;
    [Feig_vector, Feig_value] = eig(F_Manip);
    if Feig_value > max_f 
        max_f = Feig_value;
        max_theta = theta;
    end
end
disp(max_theta)