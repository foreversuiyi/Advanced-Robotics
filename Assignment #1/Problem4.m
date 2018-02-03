clear
clc
syms tx ty dz L1 W zoff pi;
T_0_1 = DH_link(-pi/2, 0, tx, L1+W);
T_1_2 = DH_link(pi/2, 0, ty, 0);
T_2_3 = DH_link(-pi/2, 0, 0, dz);
T_3_T = DH_link(0, 0, 0, zoff);
Tt = T_0_1*T_1_2*T_2_3*T_3_T;
Jac = [dz*sin(tx)*sin(ty), -dz*cos(tx)*cos(ty), -cos(tx)*sin(ty);...
       0                 , -dz*sin(ty),          cos(ty);...
       dz*cos(tx)*sin(ty), dz*sin(tx)*cos(ty),  sin(tx)*sin(ty)];
% Jac
det_Jac = det(Jac);
subs(det_Jac, ty, pi)
% Tt
% T_0_1
% T_0_1*T_1_2