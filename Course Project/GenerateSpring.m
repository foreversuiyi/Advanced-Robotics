function [x,z] = GenerateSpring(x1,z1,x2,z2,r_spring)
l_spring = sqrt((x1-x2)^2+(z1-z2)^2);
alpha  = sign(x1-x2)*(pi/2 - asin((z1-z2)/l_spring));
num = 10;
l_unit = l_spring/num;
z = 0:0.01:l_spring;
for j = 1:length(z)
    if z(j) <= l_unit
        x(j) = 0;
    elseif z(j) > l_unit && z(j) <=(num-1)*l_unit
        x(j) = r_spring*sin((z(j)-l_unit)*2*pi/l_unit);
    elseif z(j) > (num-1)*l_unit
        x(j) = 0;
    end
end

Rot = [cos(alpha) 0 sin(alpha); 0 1 0; -sin(alpha) 0 cos(alpha)];
Transl = [x1;0;z1];

for m = 1:length(z)
    P(:,m) = [x(m);0;z(m)] - [0; 0; l_spring];
    P(:,m) = Rot*P(:,m);
    P(:,m) = P(:,m) + Transl;
    x(m) = P(1,m);
    z(m) = P(3,m);
end