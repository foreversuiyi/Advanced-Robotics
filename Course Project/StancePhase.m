function [xc, zc, xc_dot, zc_dot, xf, zf] = StancePhase(ixc, izc, ixc_dot, izc_dot, ixf, izf, params)

%% Parameter Initialization
y0 = zeros(7, 1);
y0(1) = ixc;
y0(2) = izc;
y0(3) = ixc_dot;
y0(4) = izc_dot;
y0(5) = params.L;
y0(6) = atan2(ixf - ixc, izc - izf);
y0(7) = 0;

%% Time Domain Simulation
ts = params.ts;
tspan = 0:ts:5;
[t, y] = ode45(@(t,y)stance_ode(t,y,params), tspan, y0);
for k = 1:length(t)
    if y(k, 5) > params.L
        break;
    end
    xf(k) = y(k, 1) + y(k, 5)*sin(y(k, 6));
end
y = y(1:k-1, :);
xc = y(:, 1);
zc = y(:, 2);
xc_dot = y(:, 3);
zc_dot = y(:, 4);
xf = xf';
zf = y(:, 7);

xc_dot(end+1) = xc_dot(end);
zc_dot(end+1) = zc_dot(end);
xf(end+1) = xf(end) + zf(end)*tan(y(end, 6));
zf(end+1) = 0;
xc(end+1) = xf(end) - params.L*sin(y(end, 6));
zc(end+1) = zf(end) + params.L*cos(y(end, 6));

        function dydt = stance_ode(t, y, params)
        m = params.m;
        g = params.g;
        L0 = params.L;
        k1 = params.kl;
        k2 = params.kf;
        b = params.bf;
        Ft = params.Ft;
        zf_dot = -((L0-y(5))*k1*cos(y(6))+k2*y(7))/b;
        dydt = zeros(7,1);
        dydt(1) = y(3);
        dydt(2) = y(4);
        dydt(5) = -(y(3)*sin(y(6)) - y(4)*cos(y(6)) + zf_dot*cos(y(6)));
        dydt(6) = -(y(3)*cos(y(6)) + y(4)*sin(y(6)) - zf_dot*sin(y(6)))/y(5);
        dydt(7) = zf_dot;
        if dydt(5) > 0 && zf_dot > 0
            dydt(3) = -((L0 - y(5))*k1+Ft)*sin(y(6))/m;
            dydt(4) = ((L0 - y(5))*k1+Ft)*cos(y(6))/m - g;
        else
            dydt(3) = -(L0 - y(5))*k1*sin(y(6))/m;
            dydt(4) = (L0 - y(5))*k1*cos(y(6))/m - g;
        end
        
        end

end