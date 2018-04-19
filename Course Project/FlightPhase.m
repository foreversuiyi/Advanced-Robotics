function [xc, zc, xc_dot, zc_dot, xf, zf] = FlightPhase(ixc, izc, ixc_dot, izc_dot, ixf, izf, duration, params)

g = params.g;
ts = params.ts;
L = params.L;

t = 0:ts:5;
t = t';

xc = zeros(length(t), 1);
zc = zeros(length(t), 1);
xc_dot = zeros(length(t), 1);
zc_dot = zeros(length(t), 1);
xf = zeros(length(t), 1);
zf = zeros(length(t), 1);

xc(1) = ixc;
zc(1) = izc;
xc_dot(1) = ixc_dot;
zc_dot(1) = izc_dot;
xf(1) = ixf;
zf(1) = izf;

theta = -atan2(ixc-ixf,izc-izf);

xfd = ixc_dot*duration/2 + params.kd*(ixc_dot - params.dxc_dot);

thetad = asin(xfd/L);

Tf = 2*zc_dot(1)/g;
theta_dot = 2*(thetad - theta)/Tf;

upper_bound = max([thetad + theta_dot*ts, thetad - theta_dot*ts]);
lower_bound = min([thetad + theta_dot*ts, thetad - theta_dot*ts]);

swing_flag = 0;

for i = 1:length(t)-1
    
    xc(i+1) = xc(i) + xc_dot(i)*ts;
    zc(i+1) = zc(i) + zc_dot(i)*ts - g*ts^2/2;
    xc_dot(i+1) = xc_dot(i);
    zc_dot(i+1) = zc_dot(i) - g*ts;

    xf(i+1) = xc(i+1) + L*sin(theta);
    zf(i+1) = zc(i+1) - L*cos(theta);
    
    if zc(i) > L
        swing_flag = 1;
    end
    
    if swing_flag
        if theta < lower_bound || theta > upper_bound
            theta = theta + theta_dot*ts;
        else
            theta = thetad;
        end
    end
    
    if zf(i+1) < 0
        break;
    end
end

xc = xc(2:i);
zc = zc(2:i);
xc_dot = xc_dot(2:i);
zc_dot = zc_dot(2:i);
xf = xf(2:i);
zf = zf(2:i);

xc_dot(end+1) = xc_dot(end);
zc_dot(end+1) = zc_dot(end);
xf(end+1) = xf(end) + zf(end)*tan(theta);
zf(end+1) = 0;
xc(end+1) = xf(end) - L*sin(theta);
zc(end+1) = zf(end) + L*cos(theta);