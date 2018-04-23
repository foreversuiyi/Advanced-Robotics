function [pos, posd] = desiredOperTrajectory(t)
    pos = [1 + 0.4*cos(2*pi*0.5*t - 2); 1 + 0.4*sin(2*pi*0.5*t - 2)];
    posd = [-0.4*pi*sin(pi*t-2); 0.4*pi*cos(pi*t-2)];
end