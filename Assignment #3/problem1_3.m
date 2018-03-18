clear
clc
deg2ang = @(x)x(1:end)*pi/180;
ang2deg = @(x)x(1:end)*180/pi;
tf = 3;
angle = [deg2ang(15), deg2ang(75)];
acc_lim = 4*(angle(2) - angle(1))/(tf^2);
acc = [1*acc_lim, 2*acc_lim];

A_linear = [1, 0, 0, 0;
     0, 1, 0, 0;
     1, tf, tf^2, tf^3;
     0, 1, 2*tf, 3*tf^2];
B_linear = [angle(1), 0, angle(2), 0]';
para_linear = A_linear\B_linear;
ts = 0:0.001:3;
for k = 1:length(ts)
    position_linear(k) = para_linear'*[1, ts(k), ts(k)^2, ts(k)^3]';
    velocity_linear(k) = para_linear'*[0, 1, 2*ts(k), 3*ts(k)^2]';
    accelaration_linear(k) = para_linear'*[0, 0, 2, 6*ts(k)]';
end
figure(1)
set(gcf, 'color', 'white')
subplot(1,3,1); hold on
plot(ts,position_linear,'k-')
xlabel('Time [Sec]'); ylabel('Angle [Deg]');
xlim([0 3])
subplot(1,3,2); hold on
plot(ts,velocity_linear,'k-')
xlim([0 3]);
xlabel('Time [Sec]'); ylabel('Velocity [rad/sec]');
subplot(1,3,3); hold on
plot(ts,accelaration_linear,'k-')
xlim([0 3])
xlabel('Time [Sec]'); ylabel('Accelaration [rad/sec^2]');
for s = 1:2
    tb = 0.5*(tf - sqrt(tf^2 - 4*(angle(2)-angle(1))/acc(s)));
    tc = tf - tb;
    vb = acc(s)*tb;
    pb = angle(1) + 0.5*acc(s)*tb^2;
    pc = pb + vb*(tf - 2*tb);
    position_para = zeros(1, length(ts));
    velocity_para = zeros(1, length(ts));
    accelaration_para = zeros(1, length(ts));
    for k = 1:length(ts)
        if ts(k) <= tb
            position_para(k) = angle(1) + 0.5*acc(s)*ts(k)^2;
            velocity_para(k) = acc(s)*ts(k);
            accelaration_para(k) = acc(s);
        elseif ts(k) <= tc
            position_para(k) = pb + vb*(ts(k)-tb);
            velocity_para(k) = vb;
            accelaration_para(k) = 0;
        else
            position_para(k) = pc + vb*(ts(k)-tc) - 0.5*acc(s)*(ts(k)-tc)^2;
            velocity_para(k) = vb - acc(s)*(ts(k)-tc);
            accelaration_para(k) = -acc(s);
        end
    end
    if s == 1
        subplot(1,3,1);
        plot(ts, position_para, 'k--');    
        subplot(1,3,2);
        plot(ts, velocity_para, 'k--');
        subplot(1,3,3);
        plot(ts, accelaration_para, 'k--');
    else
        subplot(1,3,1);
        plot(ts, position_para, 'k-.');
        legend('Cubic', 'Blend a = 1.5*limit', 'Blend a = 3*limit');
        subplot(1,3,2);
        plot(ts, velocity_para, 'k-.');
        legend('Cubic', 'Blend a = 1.5*limit', 'Blend a = 3*limit');
        subplot(1,3,3);
        plot(ts, accelaration_para, 'k-.');
        legend('Cubic', 'Blend a = 1.5*limit', 'Blend a = 3*limit');
    end
end
