clear
clc
close all
deg2ang = @(x)x(1:end)*pi/180;
ang2deg = @(x)x(1:end)*180/pi;
tf = [3,1];
angle = [deg2ang(15), deg2ang(75); deg2ang(10), deg2ang(100)];
figure(1)
set(gcf, 'color', 'white');
for s = 1:2
    A = [1, 0, 0, 0;
         0, 1, 0, 0;
         1, tf(s), tf(s)^2, tf(s)^3;
         0, 1, 2*tf(s), 3*tf(s)^2];
    B = [angle(s,1), 0, angle(s,2), 0]';
    para = A\B;
    ts = 0:0.01:tf(s);
    position = zeros(1, length(ts));
    velocity = zeros(1, length(ts));
    accelaration = zeros(1, length(ts));
    for k = 1:length(ts)
        position(k) = para'*[1, ts(k), ts(k)^2, ts(k)^3]';
        velocity(k) = para'*[0, 1, 2*ts(k), 3*ts(k)^2]';
        accelaration(k) = para'*[0, 0, 2, 6*ts(k)]';
    end
    position(:) = ang2deg(position(:));
    % velocity = ang2deg(velocity);
    % accelaration = ang2deg(accelaration);
    if s == 1
        subplot(1,3,1); plot(ts,position,'k-'); hold on
        xlabel('Time [Sec]'); ylabel('Angle [Deg]'); xlim([0 3]);
        subplot(1,3,2); plot(ts,velocity,'k-'); hold on
        xlim([0 3]); xlabel('Time [Sec]'); ylabel('Velocity [rad/sec]');
        subplot(1,3,3); plot(ts,accelaration,'k-'); hold on
        xlim([0 3]); xlabel('Time [Sec]'); ylabel('Accelaration [rad/sec^2]');
    elseif s == 2
        subplot(1,3,1); plot(ts,position,'k--');
        legend('First Case', 'Second case');
        subplot(1,3,2); plot(ts,velocity,'k--');
        legend('First Case', 'Second case');
        subplot(1,3,3); plot(ts,accelaration,'k--');
        legend('First Case', 'Second case');
    end
end

