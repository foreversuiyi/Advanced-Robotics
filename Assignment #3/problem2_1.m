clear
clc
close all
path = [0, 2*pi, pi/2, pi];
durations = [2, 1, 2];
accs = [15, 40, 30, 5];
ts = 0:0.001:5;
ntime = length(ts);
position = path(1)*ones(1, ntime);
velocity = zeros(1, ntime);
acceleration = zeros(1, ntime);
%%
acc1 = (path(2)-path(1))/abs(path(2)-path(1))*abs(accs(1));
t1 = durations(1) - sqrt(durations(1)^2 - 2*(path(2)-path(1))/acc1);
velocity12 = (path(2)-path(1))/(durations(1)-0.5*t1);
%%
velocity23 = (path(3)-path(2))/durations(2);
acc2 = (velocity23 - velocity12)/abs(velocity23 - velocity12)*abs(accs(2));
t2 = (velocity23 - velocity12)/acc2;
%%
acc4 = (path(3)-path(4))/abs(path(3)-path(4))*abs(accs(4));
t4 = durations(3) - sqrt(durations(3)^2 + 2*(path(4)-path(3))/acc4);
velocity34 = (path(4)-path(3))/(durations(3)-0.5*t4);
acc3 = (velocity34 - velocity23)/abs(velocity34 - velocity23)*abs(accs(3));
t3 = (velocity34 - velocity23)/acc3;
%%
t12 = durations(1) - t1 - 0.5*t2;
t23 = durations(2) - 0.5*t2 - 0.5*t3;
t34 = durations(3) - t4 - 0.5*t3;
period = [t1, t1+t12, t1+t12+t2, t1+t12+t2+t23, t1+t12+t2+t23+t3, t1+t12+t2+t23+t3+t34];
last_velocity = zeros(1,6);
last_postion = zeros(1,6);
%%
deltat = 0.001;
for i = 2:ntime
    if ts(i) <= period(1)
        acceleration(i) = acc1;
    elseif ts(i) <= period(2)
        acceleration(i) = 0;
    elseif ts(i) <= period(3)
        acceleration(i) = acc2;
    elseif ts(i) <= period(4)
        acceleration(i) = 0;
    elseif ts(i) <= period(5)
        acceleration(i) = acc3;
    elseif ts(i) <= period(6)
        acceleration(i) = 0;
    else
        acceleration(i) = acc4;
    end
    velocity(i) = velocity(i-1) + acceleration(i-1)*deltat;
    position(i) = position(i-1) + velocity(i-1)*deltat;
end
figure(1)
set(gcf,'color','white')
subplot(1,3,1)
plot(ts, position, 'k-');
xlabel('Time [Sec]'); ylabel('Angle [Deg]');
subplot(1,3,2)
plot(ts, velocity, 'k-');
xlabel('Time [Sec]'); ylabel('Velocity [rad/sec]');
subplot(1,3,3)
plot(ts, acceleration, 'k-');
xlabel('Time [Sec]'); ylabel('Accelaration [rad/sec^2]');