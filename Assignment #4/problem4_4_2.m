clear; clc
numD = @(q)[2*q(2)^2 + 7/3, -2; -2, 2];
numC = @(q, q_dot)[2*q(2)*q_dot(2), 2*q(2)*q_dot(1); -2*q(2)*q_dot(1), 0];
numJv = @(q)[-sin(q(1))+q(2)*cos(q(1)), sin(q(1)); cos(q(1))+q(2)*sin(q(1)), -cos(q(1))];
numJvdot = @(q, q_dot)[(-cos(q(1))-q(2)*sin(q(1)))*q_dot(1) + q_dot(2)*cos(q(1)), cos(q(1))*q_dot(1);...
    (-sin(q(1))+q(2)*cos(q(1)))*q_dot(1) + q_dot(2)*sin(q(1)), sin(q(1))*q_dot(1)];
t = 0:0.001:5;
q1 = zeros(1, length(t));
q2 = -t.^2 + 5*t;
q1_dot = zeros(1, length(t));
q2_dot =  -2*t + 5;
q1_dou_dot = zeros(1, length(t));
q2_dou_dot = -2;
x_dou_dot = zeros(2, length(t));
x_dot = zeros(2, length(t));
x = zeros(2, length(t));
x_dot(:,1) = numJv([q1(1);q2(1)])*[q1_dot(1);q2_dot(1)];
x(:,1) = [cos(q1(1))+q2(1)*sin(q1(1)); sin(q1(1))-q2(1)*cos(q1(1))];

b1 = 0.1;
b2 = 0.5;

for k = 1:length(t)-1
    D_q = numD([q1(k);q2(k)]);
    C_q = numC([q1(k);q2(k)], [q1_dot(k);q2_dot(k)]);
    Jv = numJv([q1(k);q2(k)]);
    Jv_dot = numJvdot([q1(k);q2(k)], [q1_dot(k);q2_dot(k)]);
    q1_dou_dot(k) = D_q(1,1)\(-D_q(1,2)*q2_dou_dot-C_q(1,1)*q1_dot(k)-C_q(1,2)*q2_dot(k));
    q1_dot(k+1) = q1_dot(k) + q1_dou_dot(k)*0.001;
    q1(k+1) = q1(k) + q1_dot(k)*0.001;
    M_A = pinv(Jv*inv(D_q)*Jv.');
    C_A_xdot = M_A*Jv*inv(D_q)*C_q*[q1_dot(k);q2_dot(k)] - M_A*Jv_dot*[q1_dot(k);q2_dot(k)];
    tau = D_q*[q1_dou_dot(k);q2_dou_dot] + C_q*[q1_dot(k);q2_dot(k)];
    F = pinv(Jv.')*([b1*1*q1_dot(k); b2*2*q2_dot(k)]+tau);
    x_dou_dot(:,k) = pinv(M_A)*(F - C_A_xdot);
    x_dot(:,k+1) = x_dot(:,k) + x_dou_dot(:,k)*0.001;
    x(:,k+1) = x(:,k) + x_dot(:,k)*0.001;
end

figure(1)
set(gcf, 'color', 'white');
subplot(3,2,1);
plot(t, x(1,:), 'k'); legend('x'); grid on; xlabel('time')
subplot(3,2,2);
plot(t, x(2,:), 'k'); legend('y'); grid on; xlabel('time')
subplot(3,2,3);
plot(t, x_dot(1,:), 'k'); legend('x-dot'); grid on; xlabel('time')
subplot(3,2,4);
plot(t, x_dot(2,:), 'k'); legend('y-dot'); grid on; xlabel('time')
subplot(3,2,5);
plot(t, x_dou_dot(1,:), 'k'); legend('x-dou-dot'); grid on; xlabel('time')
subplot(3,2,6);
plot(t, x_dou_dot(2,:), 'k'); legend('y-dou-dot'); grid on; xlabel('time')