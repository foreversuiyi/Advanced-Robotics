clear; clc;
numD = @(q)[2*q(2)^2 + 7/3, -2; -2, 2];
numC = @(q, q_dot)[2*q(2)*q_dot(2), 2*q(2)*q_dot(1); -2*q(2)*q_dot(1), 0];
t = 0:0.001:5;
q_d = [sin(2*pi*t);-t.^2 + 5*t];
q_dot_d = [2*pi*cos(2*pi*t);-2*t+5];
q_dou_dot_d = [-4*pi*pi*sin(2*pi*t);-2*ones(1, length(t))];
q = zeros(2, length(t));
q_dot = zeros(2, length(t));
kp = 100;
kv = 20;
for k = 1:length(t)-1
    D_q = numD(q(:,k));
    C_q = numC(q(:,k), q_dot(:,k));
    y = q_dou_dot_d(:,k) + kv*(q_dot_d(:,k) - q_dot(:,k)) + kp*(q_d(:,k) - q(:,k));
    tau = D_q*y + C_q*q_dot(:,k);
    q_dou_dot(:,k) = D_q\(tau - C_q*q_dot(:,k));
    q_dot(:,k+1) = q_dot(:,k) + q_dou_dot(:,k)*0.001;
    q(:,k+1) = q(:,k) + q_dot(:,k)*0.001;
end
figure
set(gcf, 'color', 'white');
hold on
plot(t, q(1,:), 'r');
plot(t, q_d(1,:), 'r*');
plot(t, q(2,:), 'b');
plot(t, q_d(2,:), 'b*');
xlabel('time')
legend('real theta', 'desired theta', 'real d', 'desired d')
grid on