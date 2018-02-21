clc; clear; close all
%% Initial parameters
for s = 0:4
    theta = -(pi/12 + (5*pi/12 * s /5));
    sim_ang = [-pi/4, theta, -2*theta]';
    [link_x,link_y,link_z,Jacobian] = problem1_fkine(sim_ang);
    figure(1)
    set(gcf, 'color', 'white')
    subplot(1,2,1)
    title('Velocity Manipulability')
    hold on
    view(-45,0)
    grid on
    V_Manip = Jacobian * Jacobian';
    F_Manip = Jacobian'* Jacobian;
%% Velocity Manipulability
    [Veig_vector, Veig_value] = eig(V_Manip);
    [x,y,z] = ellipsoid(0, 0, 0, sqrt(Veig_value(1,1)), sqrt(Veig_value(2,2)),...
        sqrt(Veig_value(3,3)), 20);
    coor = [x(:), y(:), z(:)]'; coor = Veig_vector*coor;
    x = reshape(coor(1,:),size(x)) + link_x(end);
    y = reshape(coor(2,:),size(y)) + link_y(end);
    z = reshape(coor(3,:),size(z)) + link_z(end);
    rand_color = rng;
    rng(rand_color)
    surf(x,y,z, 'LineStyle', 'none', 'FaceColor', rand(1,3))
    alpha(0.25)
    zlim([-1.5,3.5])
%% Plot LINK
    plot3(link_x, link_y, link_z, 'k-', 'linewidth', 2);
    c_r = 0.05;
    for k = 1:3
        [x,y,z]=sphere(10);
        X=x*c_r+link_x(k+1);
        Y=y*c_r+link_y(k+1);
        Z=z*c_r+link_z(k+1);
        surf(X,Y,Z, 'FaceColor', [0,0,0]);
    end
%% Force Manipulability
    subplot(1,2,2)
    title('Force Manipulability')
    hold on
    view(-45,0)
    grid on
    [Feig_vector, Feig_value] = eig(F_Manip);
    [x,y,z] = ellipsoid(0, 0, 0, sqrt(Feig_value(1,1)), sqrt(Feig_value(2,2)),...
        sqrt(Feig_value(3,3)), 10);
    coor = [x(:), y(:), z(:)]'; coor = Feig_vector*coor;
    x = reshape(coor(1,:),size(x)) + link_x(end);
    y = reshape(coor(2,:),size(y)) + link_y(end);
    z = reshape(coor(3,:),size(z)) + link_z(end);
    rng(rand_color)
    surf(x,y,z, 'LineStyle', 'none', 'FaceColor', rand(1,3))
    alpha(0.25)
    zlim([-1,3])
    %% Plot LINK
    plot3(link_x, link_y, link_z, 'k-', 'linewidth', 2);
    c_r = 0.05;
    for k = 1:3
        [x,y,z]=sphere(10);
        X=x*c_r+link_x(k+1);
        Y=y*c_r+link_y(k+1);
        Z=z*c_r+link_z(k+1);
        surf(X,Y,Z, 'FaceColor', [0,0,0]);
    end
end