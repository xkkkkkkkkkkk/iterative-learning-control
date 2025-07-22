% 保存为 ilcUpdateFunction.m
function [u_k, yD_k, k_new, Useq_new] = ilcUpdateFunction(Yseq, y_k, Useq, k, gamma)
%#codegen

Nseq = numel(Yseq);
k_new = k+1;
if k_new > Nseq
    k_new = 1;
end

yD_k = Yseq(k);
u_k = Useq(k);

e_k = yD_k - y_k;

Useq_new = Useq;
k_learn = k-1;
if k_learn < 1
    k_learn = Nseq;
end

Useq_new(k_learn) = Useq_new(k_learn) + gamma*e_k;
end