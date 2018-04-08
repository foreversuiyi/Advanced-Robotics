clear; clc
numD = @(q)[2*q(2)^2 + 7/3, -2; -2, 2];
numC = @(q, q_dot)[2*q(2)*q_dot(2), 2*q(2)*q_dot(1); -2*q(2)*q_dot(1), 0];
t = 0:0.001:5;
q1 = zeros(1, length(t));
q2 = -t.^2 + 5*t;
q1_dot = zeros(1, length(t));
q2_dot =  -2*t + 5;
q1_dou_dot = zeros(1, length(t));
q2_dou_dot = -2;
for k = 1:length(t)-1
    D_q = numD([q1(k);q2(k)]);
    C_q = numC([q1(k);q2(k)], [q1_dot(k);q2_dot(k)]);
    q1_dou_dot(k) = D_q(1,1)\(-D_q(1,2)*q2_dou_dot-C_q(1,1)*q1_dot(k)-C_q(1,2)*q2_dot(k));
    q1_dot(k+1) = q1_dot(k) + q1_dou_dot(k)*0.001;
    q1(k+1) = q1(k) + q1_dot(k)*0.001;
end
figure(1)
set(gcf, 'color', 'white');
subplot(3,2,1);
plot(t, q1, 'k'); legend('theta1'); grid on; xlabel('time')
subplot(3,2,2);
plot(t, q2, 'k'); legend('d'); grid on; xlabel('time')
subplot(3,2,3);
plot(t, q1_dot, 'k'); legend('theta1-dot'); grid on; xlabel('time')
subplot(3,2,4);
plot(t, q2_dot, 'k'); legend('d-dot'); grid on; xlabel('time')
subplot(3,2,5);
plot(t, q1_dou_dot, 'k'); legend('theta1-dou-dot'); grid on; xlabel('time')
subplot(3,2,6);
plot(t, q2_dou_dot*ones(1, length(t)), 'k'); legend('d-dou-dot'); grid on; xlabel('time')