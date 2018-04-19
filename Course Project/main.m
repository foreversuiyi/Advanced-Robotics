clear
clc
params.m = 5;
params.g = 10;
params.bf = 1000;
params.kl = 1000;
params.kf = 10000;
params.Ft = 20;
params.ts = 0.01;
params.L = 1;
params.kd = 0.08;
params.dxc_dot = 1;
params.r_spring = 0.1;

ixc = 5;
izc = 0.95;
ixc_dot = 1;
izc_dot = -2;
ixf = ixc + sqrt(params.L^2 - izc^2);
izf = 0;

XC = [];
ZC = [];
XF = [];
ZF = [];

XC_dot = [];

for m = 1:70
    [Sxc, Szc, Sxc_dot, Szc_dot, Sxf, Szf] = StancePhase(ixc, izc, ixc_dot, izc_dot, ixf, izf, params);
    
    duration = params.ts * length(Sxc);
    XC = [XC; Sxc];
    ZC = [ZC; Szc];
    XF = [XF; Sxf];
    ZF = [ZF; Szf];
    XC_dot = [XC_dot; Sxc_dot];
    
    ixc = Sxc(end);
    izc = Szc(end);
    ixc_dot = Sxc_dot(end);
    izc_dot = Szc_dot(end);
    ixf = Sxf(end);
    izf = Szf(end);
    
    [Fxc, Fzc, Fxc_dot, Fzc_dot, Fxf, Fzf] = FlightPhase(ixc, izc, ixc_dot, izc_dot, ixf, izf, duration, params);
    XC = [XC; Fxc];
    ZC = [ZC; Fzc];
    XF = [XF; Fxf];
    ZF = [ZF; Fzf];
    XC_dot = [XC_dot; Fxc_dot];
    
    ixc = Fxc(end);
    izc = Fzc(end);
    ixc_dot = Fxc_dot(end);
    izc_dot = Fzc_dot(end);
    ixf = Fxf(end);
    izf = Fzf(end);
end

t = 1:length(XC);
t = params.ts*t;
figure(100)
plot(t, XC_dot);
xlabel('time (s)');
ylabel('XC-dot (m/s)');
title('XC-dot vs time');

figure(101)
plot(XC,ZC,XF,ZF)
xlabel('X (m)');
ylabel('Z (m)');
title('CoM and Foot Position');
legend('CoM position','Foot Position');

new_fig = figure;
set(gcf,'outerposition',get(0,'ScreenSize'), 'color', 'white')
for k = 1:length(XC)
    if mod(k, 5) == 0
        figure(new_fig)

        animation = subplot(3,1,1);
        cla(animation)
        % plot the ground
        xmin = 0;
        xmax = 50;
        ymin = -1;
        ymax = 5;
        axis([xmin,xmax,ymin,ymax]); %axis limit
        set(gca,'DataAspectRatio',[1 1 1]); %display ratio
        rectangle('Position', [xmin ymin xmax abs(ymin)],...
                  'Curvature', 0,...
                  'FaceColor', [1 0 1]*0.5,...
                  'EdgeColor', [0 0 0]);
        % plot the body
        center_x = XC(k);
        center_y = ZC(k);
        r_body = 0.15;
        rectangle('Position', [center_x-r_body center_y-r_body 2*r_body 2*r_body],...
                  'Curvature', 1,...
                  'FaceColor', [0 0 1],...
                  'EdgeColor', [0 0 0]);
        % plot the spring
        [x, z] = GenerateSpring(XC(k),ZC(k),XF(k),ZF(k),params.r_spring);
        line(x, z)
        axis([0,30,-0.5,2.5])

        % plot xc_dot
        subplot(3,1,2)
        title('xc-dot')
        hold on
        plot(params.ts*k, XC_dot(k), 'ko', 'markersize', 1)
        xlabel('time (s)')
        ylabel('xc-dot (m/s)')

        % plot zc
        subplot(3,1,3)
        title('zc')
        hold on
        plot(params.ts*k, ZC(k), 'ko', 'markersize', 1)
        xlabel('time (s)')
        ylabel('zc (m)')

        pause(0.0001)
    end
end

