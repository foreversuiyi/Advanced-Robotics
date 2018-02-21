%% question vi
clear; clc; close all
k0 = 250; delta_t = 0.001; t = 0:delta_t:10;
xd = 0.25*(1-cos(pi.*t)); yd = 0.25*(1-sin(pi.*t));
xd_dot = (pi*sin(pi.*t))/4; yd_dot = -(pi*cos(pi*t))/4;
K_gain = diag([500,500]); qlim = [pi/4, pi;-3*pi/4,-pi/4;-pi,0];
q1m = sum(qlim(1,:))/2; q2m = sum(qlim(2,:))/2; q3m = sum(qlim(3,:))/2;
q = [pi/2; -pi/2; -pi/2];
x = []; y = []; r = 0.025; links = [];
figure(1); xlabel('X'); ylabel('Y');
for k = 1:length(t)
    [link_x, link_y, Jacobian] = problem2_fkine(q(:,k));
    Inv_Jacobian = problem2_pinv(Jacobian);
    error = [xd(k)-link_x(end); yd(k)-link_y(end)];
    null_space = eye(3) - Inv_Jacobian*Jacobian;
    q0_dot = -[q(1,k)-q1m; q(2,k)-q2m; q(3,k)-q3m];
%   selet q_dot
    q_dot = Inv_Jacobian * (xd_dot(k) + K_gain * error) + k0 * null_space*q0_dot;
%     q_dot = Inv_Jacobian * (xd_dot(k) + K_gain * error);
    q(:, k+1) = q(:,k) + q_dot*delta_t;
    links(:,:,k) = [link_x; link_y];
    x(k) = link_x(end); y(k) = link_y(end);
end
%% Plot
figure(1); set(gcf, 'color', 'white');
subplot(1,2,2); hold on
plot(t(20:end),q(1,20:end-1), 'ko', 'MarkerSize',5)
plot(t(20:end),q(2,20:end-1), 'bo', 'MarkerSize',5)
plot(t(20:end),q(3,20:end-1), 'ro', 'MarkerSize',5)
title('Joint Angle'); xlabel('time'); ylabel('joint position');
legend('Joint 1', 'Joint 2', 'Joint 3')
plot([t(20),t(end)],[qlim(1,1),qlim(1,1)],'k-');
plot([t(20),t(end)],[qlim(1,2),qlim(1,2)],'k-');
plot([t(20),t(end)],[qlim(2,1),qlim(2,1)],'b-');
plot([t(20),t(end)],[qlim(2,2),qlim(2,2)],'b-');
plot([t(20),t(end)],[qlim(3,1),qlim(3,1)],'r-');
plot([t(20),t(end)],[qlim(3,2),qlim(3,2)],'r-');
for k = 20:length(t)
    link_x = links(1,:,k); link_y = links(2,:,k);
    subplot(1,2,1)
    plot(link_x, link_y, 'k-', 'linewidth', 2);
    hold on; axis equal
    plot(x(20:end), y(20:end), 'k-', 'linewidth', 0.5);
    for s = 1:length(link_x)
        rectangle('Position',[link_x(s)-r,link_y(s)-r,2*r,2*r],'Curvature',[1,1])
    end
    xlabel('X'); ylabel('Y');
    title('Inverse Kinematics'); hold off
    drawnow
end