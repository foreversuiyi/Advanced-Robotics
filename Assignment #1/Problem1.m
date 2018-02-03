close all; clear; clc;
% syms L t1 t2 t3 t4 t5 t6 pi;
%%Corner Case
% L = 1; t1 = 0; t2 = - pi/2; t3 = L; t4 = 0; t5 = 0; t6 = 0;
%%Sine wave
omega = 2*pi;
t = linspace(0,5,500);
sequence = sin(omega*t);
%%Loop
L = 0.5; t1 = 0; t4 = 0; t5 = 0; t6 = 0;
Trajectory = {[],[],[],[],[],[],[]};
Frame_i = {}; Frame_o = {};
for i = 1:length(t)
    t2 = pi/6*sequence(i); t3 = 0.1*sequence(i);
%% Parameters
    params = struct('L',{},'theta1',{},'theta2',{},'theta3',{},'theta4',{},...
    'theta5',{},'theta6',{});
    params(1).L = L; params.theta1 = t1; params.theta2 = t2; params.theta3 = t3;
    params.theta4 = t4; params.theta5 = t5; params.theta6 = t6;
    Frame = {};
%% Forward Kinematics
    T_0_0 = DH_link(0, 0, 0, 0); 
    Frame{1} = T_0_0; % Frame 0
    T_0_1 = DH_link(0, params.L, params.theta1, -params.L); 
    Frame{2} = Frame{1}*T_0_1; % Frame 1
    T_1_2 = DH_link(-pi/2, 0, params.theta2, params.L); 
    Frame{3} = Frame{2}*T_1_2; %Frame 2
    T_2_3 = DH_link(-pi/2, 0, pi/2, params.theta3); 
    Frame{4} = Frame{3}*T_2_3; %Frame 3
    T_3_4 = DH_link(0, params.L, params.theta4, params.L); 
    Frame{5} = Frame{4}*T_3_4; %Frame 4
    T_4_5 = DH_link(pi/2, params.L, params.theta5, 0); 
    Frame{6} = Frame{5}*T_4_5; %Frame 5
    T_5_6 = DH_link(-pi/2, params.L, params.theta6, 0); 
    Frame{7} = Frame{6}*T_5_6; %Frame 6
    T_6_7 = DH_link(pi, params.L, 0, params.L);
    Frame{8} = Frame{7}*T_6_7; %Frame tool   
    for j = 1:7
        if i == 1
            Frame_i(j) = Frame(j+1);
        end
        if i == ceil(length(t)/7)
            Frame_o(j) = Frame(j+1);
        end
        Trajectory{j} = [Trajectory{j},Frame{j+1}(1:3,4)];
    end
%% Plot Robot
    if isnumeric(Frame{8})
%%Real Time Drawing
        figure(1)
        hold on
        view(3)
        grid on
        axis square
%     axis([-5,5,-5,5,-5,5])
        [c_x, c_y, c_z] = cylinder(L/5);
        c_z = (c_z - max(c_z(:,1))/2)/10;
        plot_links = [];
        for i = 1:8
            axis_x{i} = Frame{i}(1:3, 1);
            axis_y{i} = Frame{i}(1:3, 2);
            axis_z{i} = Frame{i}(1:3, 3);
            origin{i} = Frame{i}(1:3, 4);
            plot_links = [plot_links, origin{i}];
            text(origin{i}(1),origin{i}(2),origin{i}(3),['  Frame ', num2str(i-1)]);
            quiver3(origin{i}(1), origin{i}(2), origin{i}(3), axis_x{i}(1)/5, axis_x{i}(2)/5, axis_x{i}(3)/5,...
            'color','red','linewidth',1);
            text(origin{i}(1)+axis_x{i}(1)/5,origin{i}(2)+axis_x{i}(2)/5,origin{i}(3)+axis_x{i}(3)/5,'x');
            quiver3(origin{i}(1), origin{i}(2), origin{i}(3), axis_y{i}(1)/5, axis_y{i}(2)/5, axis_y{i}(3)/5,...
            'color','green','linewidth',1);
            text(origin{i}(1)+axis_y{i}(1)/5,origin{i}(2)+axis_y{i}(2)/5,origin{i}(3)+axis_y{i}(3)/5,'y');
            quiver3(origin{i}(1), origin{i}(2), origin{i}(3), axis_z{i}(1)/5, axis_z{i}(2)/5, axis_z{i}(3)/5,...
            'color','blue','linewidth',1);
            text(origin{i}(1)+axis_z{i}(1)/5,origin{i}(2)+axis_z{i}(2)/5,origin{i}(3)+axis_z{i}(3)/5,'z');
            joint{1} = Frame{i}*[c_x(1, 1:end); c_y(1, 1:end); c_z(1, 1:end); ones(1, length(c_x(1,:)))];
            joint{2} = Frame{i}*[c_x(2, 1:end); c_y(2, 1:end); c_z(2, 1:end); ones(1, length(c_x(2,:)))];
            joint_x = [joint{1}(1,:); joint{2}(1,:)];
            joint_y = [joint{1}(2,:); joint{2}(2,:)];
            joint_z = [joint{1}(3,:); joint{2}(3,:)];
            if i~=8 && i~=1
                surf(joint_x, joint_y, joint_z);
            end
            shading interp
            alpha(.5);
        end
        plot3(plot_links(1,:),plot_links(2,:),plot_links(3,:),'k-','linewidth',3);
        drawnow
        clf;
    end
end
figure(2)
set(gcf, 'color', 'w')
for i = 1:7
    subplot(4,2,i)
    if i == 7
        title('Tip Frame');
    else
        title(['Frame',num2str(i)]);
    end 
    hold on
    view(3);
    axis_x = [Frame_i{i}(1:3, 1),Frame_o{i}(1:3, 1)]/5;
    axis_y = [Frame_i{i}(1:3, 2),Frame_o{i}(1:3, 2)]/5;
    axis_z = [Frame_i{i}(1:3, 3),Frame_o{i}(1:3, 3)]/5;
    origin = [Frame_i{i}(1:3, 4),Frame_o{i}(1:3, 4)];
    plot3(Trajectory{i}(1,:),Trajectory{i}(2,:),Trajectory{i}(3,:),'k-','linewidth',2);
    legend('Trajectory')
    for j = 1:2
        if j == 1
            if i == 1 || i == 2
                text(origin(1,j),origin(2,j),origin(3,j),'      Start');
            else
                text(origin(1,j),origin(2,j),origin(3,j),' Start');
            end
            quiver3(origin(1,j), origin(2,j), origin(3,j), axis_x(1,j), axis_x(2,j), axis_x(3,j),...
            'color','red','linewidth',1);
            text(origin(1,j)+axis_x(1,j),origin(2,j)+axis_x(2,j),origin(3,j)+axis_x(3,j),'x');
            quiver3(origin(1,j), origin(2,j), origin(3,j), axis_y(1,j), axis_y(2,j), axis_y(3,j),...
            'color','green','linewidth',1);
            text(origin(1,j)+axis_y(1,j),origin(2,j)+axis_y(2,j),origin(3,j)+axis_y(3,j),'y');
            quiver3(origin(1,j), origin(2,j), origin(3,j), axis_z(1,j), axis_z(2,j), axis_z(3,j),...
            'color','blue','linewidth',1);
            text(origin(1,j)+axis_z(1,j),origin(2,j)+axis_z(2,j),origin(3,j)+axis_z(3,j),'z');            
        else
            text(origin(1,j),origin(2,j),origin(3,j),' In');
            quiver3(origin(1,j), origin(2,j), origin(3,j), axis_x(1,j), axis_x(2,j), axis_x(3,j),...
            'color','red','linewidth',1,'LineStyle','--');
            text(origin(1,j)+axis_x(1,j),origin(2,j)+axis_x(2,j),origin(3,j)+axis_x(3,j),'x');
            quiver3(origin(1,j), origin(2,j), origin(3,j), axis_y(1,j), axis_y(2,j), axis_y(3,j),...
            'color','green','linewidth',1,'LineStyle','--');
            text(origin(1,j)+axis_y(1,j),origin(2,j)+axis_y(2,j),origin(3,j)+axis_y(3,j),'y');
            quiver3(origin(1,j), origin(2,j), origin(3,j), axis_z(1,j), axis_z(2,j), axis_z(3,j),...
            'color','blue','linewidth',1,'LineStyle','--');
            text(origin(1,j)+axis_z(1,j),origin(2,j)+axis_z(2,j),origin(3,j)+axis_z(3,j),'z');
        end
    end
    grid on;
end
%% Calculate Jacobian
JacW = []; JacV = [];
for i = 1:6
    JacW = [JacW, Frame{i+1}(1:3, 3)];
    JacV = [JacV, cross(Frame{i+1}(1:3, 3), (Frame{8}(1:3, 4) - Frame{i+1}(1:3, 4)))];
end
JacW(1:3, 3) = zeros(3, 1);
JacV(1:3, 3) = Frame{4}(1:3, 3);
Jac = [JacV; JacW];
det_Jac = det(Jac);
% for i = 1:6
%     for j = 1:6
%         disp(['Jac(',num2str(i),',',num2str(j),') = ']);
%         disp(Jac(i,j));
%     end
% end
q_dot = [1; 0; 1; 0; 0; 1];
Cat_vw = Jac*q_dot;
%%