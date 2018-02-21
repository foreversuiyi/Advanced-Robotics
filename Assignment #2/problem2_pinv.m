function out_inv = problem2_pinv(input)
[U,S,V] = svd(input);
T=S;
T(S~=0) = 1./S(S~=0);
out_inv = V * T' * U';