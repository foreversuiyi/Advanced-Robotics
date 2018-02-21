clear; clc; close all
delta_t = 0.001; t = 0:delta_t:10;
xd = 0.25*(1-cos(pi.*t)); yd = 0.25*(1-sin(pi.*t)); 
xd_dot = (pi*sin(pi.*t))/4; yd_dot = -(pi*cos(pi*t))/4;
K_gain = diag([500,500]);
q = [pi/2; -pi/2; -pi/2]; x = []; y = []; r = 0.025; links = []; q_dot_s = [];
W = diag([1, 1, 1]);
for k = 1:length(t)
    [link_x, link_y, Jacobian] = problem2_fkine(q(:,k));
    Inv_Jacobian = W\Jacobian'*problem2_pinv(Jacobian/W*Jacobian');
    error = [xd(k)-link_x(end); yd(k)-link_y(end)];
    q_dot = Inv_Jacobian * (xd_dot(k) + K_gain * error);
    q_dot_s(:,k) = q_dot;
    q(:, k+1) = q(:,k) + q_dot*delta_t;
    links(:,:,k) = [link_x; link_y];
    x(k) = link_x(end);
    y(k) = link_y(end);
end
%% Plot
figure(1); subplot(1,2,1); hold on; xlabel('time'); ylabel('angular velocity');
plot(t(20:end),q_dot_s(1,20:end),'r-');
plot(t(20:end),q_dot_s(2,20:end),'g-');
plot(t(20:end),q_dot_s(3,20:end),'k-');
legend('q1 dot', 'q2 dot', 'q3 dot')
figure(1); subplot(1,2,2); set(gcf, 'color', 'white');
for k = 20:length(t)
    link_x = links(1,:,k); link_y = links(2,:,k);
    plot(link_x, link_y, 'k-', 'linewidth', 2);
    hold on; axis equal
    plot(x(20:end), y(20:end), 'k-', 'linewidth', 0.5);
    for s = 1:length(link_x)
        rectangle('Position',[link_x(s)-r,link_y(s)-r,2*r,2*r],'Curvature',[1,1])
    end
    xlabel('X'); ylabel('Y');
    title('Inverse Kinematics');
    drawnow; hold off
end