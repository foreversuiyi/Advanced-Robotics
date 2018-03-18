clear
clc
close all
x0 = [0; 0];
tspan = [0 10];
[t, x] = ode45(@(t,x)problem4_fun(t,x), tspan, x0);
figure(1)
set(gcf, 'color', 'white')
plot(t, x(:,1), 'k-')
hold on
plot(t, 0.1*ones(1, length(t)), 'k--')
plot(t, 0.11*ones(1, length(t)), 'k--')
xlabel('time');
ylabel('theta');
legend('step response');