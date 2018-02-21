clear
clc
close all
%% question i
q = [pi, -pi/2, pi/2];
[~,~,Jacobian] = problem2_fkine(q);
%% question ii
q1_dot = [pi/4, 0, 0]'; q2_dot = [0, pi/4, 0]'; q3_dot = [0, 0, pi/4]';
v1 = Jacobian * q1_dot; v2 = Jacobian * q2_dot; v3 = Jacobian * q3_dot;
%% question iii
null_space = eye(3) - problem2_pinv(Jacobian)*Jacobian;
q1_null_dot = null_space*q1_dot; q2_null_dot = null_space*q2_dot;
q3_null_dot = null_space*q3_dot;
v1_null = Jacobian * q1_null_dot; v2_null = Jacobian * q2_null_dot;
v3_null = Jacobian * q3_null_dot;
%% question iv
[U, S, V] = svd(Jacobian);
null_space_svd(:,1) = V(:,end);
null_space_svd(:,2) = -V(:,end);
null_space_svd(:,3) = -V(:,end);