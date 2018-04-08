function C = CalC(D, q, q_dot)
%% Calculate symbolic C(q,q_dot) based on D
dim = size(D,1);
c = sym(zeros(dim,dim,dim));
for i = 1:dim
    for j = 1:dim
        for k = 1:dim
            c(i,j,k) = 1/2*(diff(D(k,j),q{i})+diff(D(k,i),q{j})-diff(D(i,j),q{k}));
        end
    end
end
C = sym(zeros(dim,dim));
for k = 1:dim
    for j = 1:dim
        for i = 1:dim
            C(k,j) = C(k,j) + c(i,j,k)*q_dot{i};
        end
    end
end