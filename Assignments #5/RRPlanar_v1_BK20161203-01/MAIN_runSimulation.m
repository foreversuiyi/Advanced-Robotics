clear; clear global; close all;

%%%% Two Link RR Planar model & enviroment parameters
%%%% RR means revolute joint-revolute joint
model = RRPlanarModel();
% For the damping terms, to create some noticable decceleration effects, 
% b1, b2 may need to set to 30 or above
model.b1 = 20; % dumper of joint 1, 0 => no friction
model.b2 = 20; % dumper of joint 2, 0 => no friction
model.wm = 1; % disturbance in M term, 1 => no disturbance
model.wc = 1; % disturbance in C term, 1 => no disturbance
model.wg = 1; % disturbance in G term, 1 => no disturbance

%%%% Define inital state
% 
% the state format is: [q1; q2; q1d; q2d]
%
x0 = [deg2rad(0); deg2rad(0); deg2rad(0); deg2rad(0)]; 
% x0 = [deg2rad(90); deg2rad(180); deg2rad(90); deg2rad(180)];
% x0 = [deg2rad(90); deg2rad(0); deg2rad(0); deg2rad(0)]; 

%%%% Define time span for simulation
% e.g. tspan = [0 10]; => for simulation from t=0 to t=10 seconds. 
tspan = [0 15];

%%%% Control law switching
%
% controlLaw = 'Passive';
% controlLaw = 'PD';
% controlLaw = 'ComputedTorque';
controlLaw = 'JacobianTranspose';
% controlLaw = 'ComplianceControl';


%%%% Run simulation
[t,x] = runRRPlanarSim(model, x0, controlLaw, tspan);

%%%% Run animation
runAnimation(model, tspan, t, x, []);

%%%% Show plot
if strcmp(controlLaw, 'JacobianTranspose')
    %% Jacobian Transpose control plot
    figure;
    set(gcf, 'color', 'white')
    hold on;
    plot(t, rad2deg(x(1,:)), '-r', 'LineWidth',1);
    plot(t, rad2deg(x(2,:)), '-b', 'LineWidth',1);
    xlabel('t');
    ylabel('Joint angle (degree)');
    title('Joint Angle Trajectory');
    legend('q_1', 'q_2');
    hold off;

    %%%% Show plot - Velocity
    figure;
    set(gcf, 'color', 'white')
    hold on;
    plot(t, rad2deg(x(3,:)), '-r', 'LineWidth',1);
    plot(t, rad2deg(x(4,:)), '-b', 'LineWidth',1);
    xlabel('t');
    ylabel('angular velocity (degree/s)');
    title('Joint Angular Velocity');
    legend('q_1', 'q_2');
    hold off;

    %%%% Show Trajectory
    pos = zeros(2, length(t));
    des_pos = pos;
    x_error = pos;
    y_error = pos;
    for k = 1:length(t)
        [~, pos(:, k)] = RRPlanarFKine(model, x(1:2, k));
        [des_pos(:, k), ~] = desiredOperTrajectory(t(k));
        x_error(k) = pos(1,k)-des_pos(1,k);
        y_error(k) = pos(2,k)-des_pos(2,k);
    end
    figure
    set(gcf, 'color', 'white')
    plot(pos(1,:), pos(2,:))
    hold on 
    plot(des_pos(1,:), des_pos(2,:), '--k')
    legend('Real Trajectory', 'Desired Trajectory')
    xlabel('x')
    ylabel('y')
    title('Operational Space')
    axis([0.5, 2, 0.5, 2])
    hold off

    %%%% Show Error
    figure
    set(gcf, 'color', 'white')
    hold on 
    plot(t, x_error, 'r');
    plot(t, y_error, 'b')
    plot(t, 0.1*ones(1, length(t)), '--k')
    plot(t, -0.1*ones(1, length(t)), '--k')
    legend('Tracking Error')
    xlabel('time')
    ylabel('error (m)')
    title('Tracking Error')
    hold off
elseif strcmp(controlLaw, 'ComplianceControl')
    figure;
    set(gcf, 'color', 'white')
    hold on;
    plot(t, rad2deg(x(1,:)), '-r', 'LineWidth',1);
    plot(t, rad2deg(x(2,:)), '-b', 'LineWidth',1);
    xlabel('t');
    ylabel('Joint angle (degree)');
    title('Joint Angle Trajectory');
    legend('q_1', 'q_2');
    hold off;

    %%%% Show plot - Velocity
    figure;
    set(gcf, 'color', 'white')
    hold on;
    plot(t, rad2deg(x(3,:)), '-r', 'LineWidth',1);
    plot(t, rad2deg(x(4,:)), '-b', 'LineWidth',1);
    xlabel('t');
    ylabel('angular velocity (degree/s)');
    title('Joint Angular Velocity');
    legend('q_1', 'q_2');
    hold off;

    %%%% Show Trajectory
    pos = zeros(2, length(t));
    des_pos = pos;
    x_error = pos;
    y_error = pos;
    for k = 1:length(t)
        [~, pos(:, k)] = RRPlanarFKine(model, x(1:2, k));
        [des_pos(:, k), ~] = desiredOperTrajectory(t(k));
        x_error(k) = pos(1,k)-des_pos(1,k);
        y_error(k) = pos(2,k)-des_pos(2,k);
    end
    figure
    set(gcf, 'color', 'white')
    plot(pos(1,:), pos(2,:))
    hold on 
    plot(des_pos(1,:), des_pos(2,:), '--k')
    legend('Real Trajectory', 'Desired Trajectory')
    xlabel('x')
    ylabel('y')
    title('Operational Space')
    axis([0.5, 2, 0.5, 2])
    hold off

    %%%% Show Error
    figure
    set(gcf, 'color', 'white')
    hold on 
    plot(t, x_error, 'r');
    plot(t, y_error, 'b')
    plot(t, 0.1*ones(1, length(t)), '--k')
    plot(t, -0.1*ones(1, length(t)), '--k')
    legend('Tracking Error x', 'Tracking Error y')
    xlabel('time')
    ylabel('error (m)')
    title('Tracking Error')
    hold off
else
    [q, qd, qdd] = desiredJointTrajectory(t);
    showPlot(t,x,[q;qd;qdd]);
end