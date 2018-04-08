clear; clc
%% Symbolic Calculation
dim = 2; q = {}; q_dot = {};
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
I1 = diag([0;4;4]);
I2 = diag([4;4;0]);
D = 2*(transpose(Jv1)*Jv1 + transpose(Jv2)*Jv2) + transpose(Jw1)*R_0_1*I1*transpose(R_0_1)*Jw1 + ...
    transpose(Jw2)*R_0_2*I2*transpose(R_0_2)*Jw2;
D = simplify(D);
C = CalC(D,q,q_dot);
C = simplify(C);
%% Numerical Calculation
numD = matlabFunction(D); % q2
numC = matlabFunction(C); % q2 q1_dot q2_dot
numG = @(q2)[0;20*sin(q2)];
ts = 0.01;
t = 0:ts:5;
%% Euler Integration
q = zeros(2, length(t));
q(:,1) = [0;pi/2];
q_dot = zeros(2, length(t));
for k = 1:length(t)-1
    D_q = numD(q(2,k));
    C_q = numC(q(2,k), q_dot(1,k), q_dot(2,k));
    G_q = numG(q(2,k));
    q_dou_dot(:,k) = D_q\(-C_q*q_dot(:,k)-G_q);
    q_dot(:,k+1) = q_dot(:,k) + q_dou_dot(:,k)*ts;
    q(:,k+1) = q(:,k) + q_dot(:,k)*ts;
end
%% ODE45
problem2ode = @(y, numD, numC, numG)[y(3);y(4);numD(y(2))\(-numC(y(2), y(3), y(4))*y(3:4)-numG(y(2)))];
[t,y] = ode45(@(t,y)problem2ode(y, numD, numC, numG), t, [0 pi/2 0 0]);
%% PLOT
figure(1)
set(gcf, 'color', 'white');
subplot(2,2,1); hold on
plot(t, q(1,:), 'k'); 
plot(t, y(:,1), 'k--'); legend('theta1 Euler', ' theta1 ODE45'); grid on; xlabel('time')
subplot(2,2,2); hold on
plot(t, q(2,:), 'k'); 
plot(t, y(:,2), 'k--'); legend('theta2 Euler', ' theta2 ODE45'); grid on; xlabel('time')
subplot(2,2,3); hold on
plot(t, q_dot(1,:), 'k'); 
plot(t, y(:,3), 'k--'); legend('theta1-dot Euler', ' theta1-dot ODE45'); grid on; xlabel('time')
subplot(2,2,4); hold on
plot(t, q_dot(2,:), 'k'); legend('theta2-dot'); grid on; xlabel('time')
plot(t, y(:,4), 'k--'); legend('theta2-dot Euler', ' theta2-dot ODE45'); grid on; xlabel('time')
