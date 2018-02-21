%% question v
clear; clc; clf; syms q1 q2 q3; k0 = 250; q_sym = [q1 q2 q3];
Jacobian_sym = problem2_fkine_sym(q_sym);
wq_manip = -sqrt(det(Jacobian_sym*Jacobian_sym'));
q0_dot_manip_sym = matlabFunction(simplify([diff(wq_manip, q1), diff(wq_manip, q2), diff(wq_manip, q3)]'));
delta_t = 0.001; t = 0:delta_t:10;
xd = 0.25*(1-cos(pi.*t)); yd = 0.25*(1-sin(pi.*t));
xd_dot = (pi*sin(pi.*t))/4; yd_dot = -(pi*cos(pi*t))/4;
K_gain = diag([500,500]); q = [pi/2; -pi/4; -pi/4];
x = []; y = []; r = 0.025; links = [];
sita = 0:pi/20:2*pi;
for s = 1:2
    for k = 1:length(t)
        [link_x, link_y, Jacobian] = problem2_fkine(q(:,k));
        Inv_Jacobian = problem2_pinv(Jacobian);
        error = [xd(k)-link_x(end); yd(k)-link_y(end)];
        null_space = eye(3) - Inv_Jacobian*Jacobian;
        if s ==1
            q_dot = Inv_Jacobian * (xd_dot(k) + K_gain * error);
        else
            q0_dot = -q0_dot_manip_sym(q(2,k),q(3,k));
            q_dot = Inv_Jacobian * (xd_dot(k) + K_gain * error) + k0*null_space*q0_dot;
        end
        q(:, k+1) = q(:,k) + q_dot*delta_t;
        x(k) = link_x(end); y(k) = link_y(end);
        links(:,:,k,s) = [link_x; link_y];
        V_manip(:,:,k,s) = Jacobian*Jacobian';
        Manipulability(k) = sqrt(det(V_manip(:,:,k,s)));
    end
    figure(1);  set(gcf, 'color', 'white'); subplot(1,3,3);  hold on
    plot(t(25:end),Manipulability(25:end)); 
end
title('Manipulability'); xlabel('t'); ylabel('Manipulability');
legend('Original Manipulability', 'Maximized Manipulability');
conf_x = []; conf_y = []; conf_ellip_x = []; conf_ellip_y = [];
conf_num = 0;
for k = 20:length(t)
    if mod(k, 400) == 0
        conf_num = conf_num + 1;
    end
    for s = 1:2
        link_x = links(1,:,k,s); link_y = links(2,:,k,s);
        [eig_vector, eig_value] = eig(V_manip(:,:,k,s));
        transform = [eig_vector, [link_x(end);link_y(end)];0,0,1];
        a_ellip = sqrt(eig_value(1,1)); b_ellip = sqrt(eig_value(2,2));
        x_ellip = a_ellip*cos(sita); y_ellip = b_ellip*sin(sita);
        ellip = transform*[x_ellip;y_ellip;ones(1,length(x_ellip))];
        x_ellip = ellip(1,:); y_ellip = ellip(2,:);
        if mod(k, 400) == 0
            conf_x(conf_num,:,s) = link_x; conf_y(conf_num,:,s) = link_y;
            conf_ellip_x(conf_num,:,s) = x_ellip; 
            conf_ellip_y(conf_num,:,s) = y_ellip;
        end
        subplot(1,3,s); 
        plot(link_x, link_y, 'k-', 'linewidth', 2);
        axis([-2, 2, -1.5, 2.5])
        xlabel('X'); ylabel('Y'); 
        hold on;
        if s ==1
            title('Original Simulation');
        else
            title('Maximized Manipulability');
        end
        plot(x(25:end), y(25:end), 'k-', 'linewidth', 0.5);
        for m = 1:length(link_x)
            rectangle('Position',[link_x(m)-r,link_y(m)-r,2*r,2*r],'Curvature',[1,1])
        end
        plot(x_ellip,y_ellip,'r-','linewidth',0.5); 
        if conf_num > 0
            for n = 1:conf_num
                plot(conf_x(n,:,s),conf_y(n,:,s),'--','linewidth', 1,'color',...
                    [1.1-0.2*n,0.3+0.1*n,0.2+0.15*n,]);
                plot(conf_ellip_x(n,:,s),conf_ellip_y(n,:,s),'--','linewidth',0.2, 'color',...
                    [1.1-0.2*n,0.3+0.1*n,0.2+0.15*n,]);
                for nn = 1:length(link_x)
                    rectangle('Position',[conf_x(n,nn,s)-r,conf_y(n,nn,s)-r,2*r,2*r],'Curvature',[1,1]);
                end
            end
        end
        hold off;
    end
    if conf_num >= 5
        break;
    end
    if mod(k, 10) == 0
        pause(0.0001)
    end
end
