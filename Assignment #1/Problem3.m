clear
clc
syms alpha beta gama;
T1=T_rotate('z', alpha);
T2=T_rotate('x',beta);
T3=T_rotate('z', gama);
Tt=T1*T2*T3;
R = [-sqrt(3)/4, -1/2, 3/4; 1/4, -sqrt(3)/2, -sqrt(3)/4; sqrt(3)/2, 0, 1/2];
a = atan2(R(1,3), -R(2,3));
c = atan2(R(3,1), R(3,2));
b = atan2(sqrt(R(3,1)^2+R(3,2)^2), R(3,3));
R_c = double(subs(Tt, {alpha, beta, gama}, {a, b, c}));
R_c = R_c(1:3,1:3);
R
R_c

Jac=[0, cos(a), sin(a)*sin(b);0, sin(a), -cos(a)*sin(b); 1, 0, cos(b)];
angle_dot = [1/180, 1/180, 1/180]';
omega = Jac * angle_dot;
omega