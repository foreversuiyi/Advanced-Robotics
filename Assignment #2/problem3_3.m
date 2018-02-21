clear; clc
q = [pi, -pi/2, pi/2];
Force = [1 0]';
[link_x,link_y,Jacobian] = problem2_fkine(q);
Inv_Jacobian_trans = pinv(Jacobian');
null_space = eye(2) - Inv_Jacobian_trans*Jacobian';