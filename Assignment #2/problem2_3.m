clear; clc; close all
delta_t = 0.001; t = 0:delta_t:10;
xd = 0.25*(1-cos(pi.*t)); yd = 0.25*(1-sin(pi.*t)); 
xd_dot = (pi*sin(pi.*t))/4; yd_dot = -(pi*cos(pi*t))/4;
magnitude = [0.5, 1, 5, 10];
for m = 1:length(magnitude)
    K_gain = diag([500,500])*magnitude(m);
    q = [pi/2; -pi/2; -pi/2];
    x = []; y = []; p_error = []; r = 0.025;
    for k = 1:length(t)
        [link_x, link_y, Jacobian] = problem2_fkine(q(:,k));
        Inv_Jacobian = problem2_pinv(Jacobian);
        error = [xd(k)-link_x(end); yd(k)-link_y(end)];
        q_dot = Inv_Jacobian * (xd_dot(k) + K_gain * error);
        q(:, k+1) = q(:,k) + q_dot*delta_t;
        x(k) = link_x(end); y(k) = link_y(end); p_error(:,k) = error;
    end
    figure(1);set(gcf, 'color', 'white');
    subplot(4,2,2*(m-1)+1); 
    plot(t(10:end), p_error(1,10:end));
    ylabel('x error'); xlabel('time');
    title(['magnitude = ', num2str(magnitude(m))]);
    subplot(4,2,2*(m-1)+2); 
    plot(t(10:end), p_error(2,10:end));
    ylabel('y error'); xlabel('time');
    title(['magnitude = ', num2str(magnitude(m))]);
end