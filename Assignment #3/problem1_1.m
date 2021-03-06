clear
clc
deg2ang = @(x)x(1:end)*pi/180;
ang2deg = @(x)x(1:end)*180/pi;
tf = 3;
angle = [deg2ang(15), deg2ang(75)];
A = [1, 0, 0, 0;
     0, 1, 0, 0;
     1, tf, tf^2, tf^3;
     0, 1, 2*tf, 3*tf^2];
B = [angle(1), 0, angle(2), 0]';
para = A\B;
ts = 0:0.01:3;
for k = 1:length(ts)
    position(k) = para'*[1, ts(k), ts(k)^2, ts(k)^3]';
    velocity(k) = para'*[0, 1, 2*ts(k), 3*ts(k)^2]';
    accelaration(k) = para'*[0, 0, 2, 6*ts(k)]';
end
position = ang2deg(position);
% velocity = ang2deg(velocity);
% accelaration = ang2deg(accelaration);
figure(1)
set(gcf, 'color', 'white')
subplot(1,3,1)
plot(ts,position,'k-')
xlabel('Time [Sec]'); ylabel('Angle [Deg]');
xlim([0 3])
subplot(1,3,2)
plot(ts,velocity,'k-')
xlim([0 3])
xlabel('Time [Sec]'); ylabel('Velocity [rad/sec]');
subplot(1,3,3)
plot(ts,accelaration,'k-')
xlim([0 3])
xlabel('Time [Sec]'); ylabel('Accelaration [rad/sec^2]');
