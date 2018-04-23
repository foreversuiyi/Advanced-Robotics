function showPlot(t, x, x_dsr)

    %%%% Show plot - Trajectory
    figure;
    set(gcf, 'color', 'white')
    hold on;
    plot(t, rad2deg(x(1,:)), '-r', 'LineWidth',1);
    plot(t, rad2deg(x(2,:)), '-b', 'LineWidth',1);
    plot(t, rad2deg(x_dsr(1,:)), ':r', 'LineWidth',1);
    plot(t, rad2deg(x_dsr(2,:)), ':b', 'LineWidth',1);
    xlabel('t');
    ylabel('Joint angle (degree)');
    title('Joint Angle Trajectory');
    legend('q_1', 'q_2','desired q_1', 'desired q_2');
    hold off;

    %%%% Show plot - Velocity
    figure;
    set(gcf, 'color', 'white')
    hold on;
    plot(t, rad2deg(x(3,:)), '-r', 'LineWidth',1);
    plot(t, rad2deg(x(4,:)), '-b', 'LineWidth',1);
    plot(t, rad2deg(x_dsr(3,:)), ':r', 'LineWidth',1);
    plot(t, rad2deg(x_dsr(4,:)), ':b', 'LineWidth',1);
    xlabel('t');
    ylabel('angular velocity (degree/s)');
    title('Joint Angular Velocity');
    legend('q_1', 'q_2', 'desired q_1', 'desired q_2');
    hold off;
    
    %%%% Show plot - Joint error
    figure;
    set(gcf, 'color', 'white')  
    hold on;
    plot(t, rad2deg(x_dsr(1,:)) - rad2deg(x(1,:)), '-r', 'LineWidth', 1);
    plot(t, rad2deg(x_dsr(2,:)) - rad2deg(x(2,:)), '-b', 'LineWidth', 1);
    plot(t, ones(1, length(t)), '--k');
    plot(t, -1*ones(1, length(t)), '--k');
    xlabel('t');
    ylabel('Joint angle error (degree)');
    title('Joint Angle Error');
    legend('Error of q1', 'Error of q2');
    hold off;

end