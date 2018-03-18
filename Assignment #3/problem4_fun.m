function dxdt = problem4_fun(t, x)
dxdt = zeros(2,1);
dxdt(1) = x(2);
% dxdt(2) = -5*x(1)-2*sqrt(5)*x(2)+0.5; % problem4_2
dxdt(2) = -5*x(1)-2*sqrt(5)*x(2)+0.5+0.05; % problem4_3
