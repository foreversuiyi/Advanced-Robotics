clear
clc
dim = 2;
q = {};
q_dot = {};
for i = 1:dim
    syms (['q',num2str(i)]);
    syms (['q',num2str(i),'_dot']);
    eval(['q{end+1} = q',num2str(i),';']);
    eval(['q_dot{end+1} = q',num2str(i),'_dot',';']);
end
T_0_1 = [cos(q1), -sin(q1), 0, cos(q1);...
         sin(q1), cos(q1),  0, sin(q1);...
         0, 0, 1, 0;...
         0, 0, 0, 1];
T_1_2 = [1, 0, 0, 0;...
         0, cos(q2), -sin(q2), sin(q2);...
         0, sin(q2), cos(q2), -cos(q2);...
         0, 0, 0, 1];
T_0_2 = T_0_1*T_1_2;

R_0_1 = T_0_1(1:3,1:3);
R_0_2 = T_0_2(1:3,1:3);

Jv1 = [cross([0;0;1], T_0_1(1:3,4)), zeros(3,1)];
Jw1 = [0,0;0,0;1,0];
Jv2 = [cross([0;0;1], T_0_2(1:3,4)), cross(T_0_1(1:3,1), T_0_2(1:3,4) - T_0_1(1:3,4))];
Jw2 = [[0;0;1], T_0_1(1:3,1)];
m = 2;
I1 = diag([0;4;4]);
I2 = diag([4;4;0]);
D = m*(transpose(Jv1)*Jv1 + transpose(Jv2)*Jv2) + transpose(Jw1)*R_0_1*I1*transpose(R_0_1)*Jw1 + ...
    transpose(Jw2)*R_0_2*I2*transpose(R_0_2)*Jw2;
D = simplify(D);
C = CalC(D,q,q_dot);
C = simplify(C);
