dim = 2;
q = {};
q_dot = {};
for i = 1:dim
    syms (['q',num2str(i)]);
    syms (['q',num2str(i),'_dot']);
    eval(['q{end+1} = q',num2str(i),';']);
    eval(['q_dot{end+1} = q',num2str(i),'_dot',';']);
end
syms m1 m2 L I
Jv1 = L/2*[-sin(q1), 0; cos(q1), 0];
Jv2 = [-L*sin(q1)+q2*cos(q1), sin(q1); L*cos(q1)+q2*sin(q1), -cos(q1)];
Jw1 = [1,0];
D = m1*transpose(Jv1)*Jv1 + m2*transpose(Jv2)*Jv2 + I*transpose(Jw1)*Jw1;
C = CalC(D,q,q_dot);
D = simplify(D);
C = simplify(C);