clear; clc
%% Symbolic Calculation
dim = 2; q = {}; q_dot = {};
for i = 1:dim
    syms (['q',num2str(i)]);
    syms (['q',num2str(i),'_dot']);
    eval(['q{end+1} = q',num2str(i),';']);
    eval(['q_dot{end+1} = q',num2str(i),'_dot',';']);
end
T_0_1 = [cos(q1), -sin(q1), 0, cos(q1); sin(q1), cos(q1),  0, sin(q1);...
         0, 0, 1, 0; 0, 0, 0, 1];
T_1_2 = [1, 0, 0, 0; 0, cos(q2), -sin(q2), sin(q2);...
         0, sin(q2), cos(q2), -cos(q2); 0, 0, 0, 1];
T_0_2 = T_0_1*T_1_2;
R_0_1 = T_0_1(1:3,1:3); R_0_2 = T_0_2(1:3,1:3);
Jv1 = [cross([0;0;1], T_0_1(1:3,4)), zeros(3,1)];
Jw1 = [0,0; 0,0; 1,0];
Jv2 = [cross([0;0;1], T_0_2(1:3,4)), cross(T_0_1(1:3,1), T_0_2(1:3,4) - T_0_1(1:3,4))];
Jw2 = [[0;0;1], T_0_1(1:3,1)];
I1 = diag([0;4;4]); I2 = diag([4;4;0]);
D = 2*(transpose(Jv1)*Jv1 + transpose(Jv2)*Jv2) + transpose(Jw1)*R_0_1*I1*transpose(R_0_1)*Jw1...
    + transpose(Jw2)*R_0_2*I2*transpose(R_0_2)*Jw2;
D = simplify(D);
% G = [0; 20*sin(q2)];
G = [0; 0];
D = double(subs(D, {q1, q2, q1_dot, q2_dot}, {pi/2, pi/2, 0, 0}));
G = double(subs(G, {q1, q2, q1_dot, q2_dot}, {pi/2, pi/2, 0, 0}));
Jv2 = double(subs(Jv2, {q1, q2, q1_dot, q2_dot}, {pi/2, pi/2, 0, 0}));
q1 = pi/2; q2 = pi/2; q1_dot = 0; q2_dot = 0;
P = [cos(q1) - sin(q1)*sin(q2); sin(q1) + cos(q1)*sin(q2); -cos(q2)];
A = (pinv(Jv2)).'*D.'*D*pinv(Jv2);
T = -Jv2*inv(D)*G;
[U,S,V] = svd(A);
a = S(1,1); 
b = S(2,2);
t = 0:0.01:2*pi;
for i = 1:length(t)    
    x(i) = 0;
    y(i) = 0.1*a*sin(t(i));
    z(i) = 0.1*b*cos(t(i));
    r = U*[0;y(i);z(i)];
    y(i) = r(2);
    z(i) = r(3);
    x(i) = x(i) + P(1) + T(1);
    y(i) = y(i) + P(2) + T(2);
    z(i) = z(i) + P(3) + T(3);
end
figure; set(gcf, 'color', 'white'); plot3([0,0,-1],[0,1,1],[0,0,0],'o-');
hold on; plot3(x,y,z); plot3(P(1) + T(1),P(2) + T(2),P(3) + T(3),'*');