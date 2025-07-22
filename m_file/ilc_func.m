function [u_k,yD_k,k_new,Useq_new] = ilc_func(Yseq,y_k,Useq,k,gamma)
% Yseq：期望输出轨迹
% y_k：实际输出
% Useq：当前控制输入
% k：时间索引
% gamma：学习增益
% u_k：当前控制输入
% yD_k：当前期望输出
% k_new：更新后时间索引
% Useq_new：更新后控制输入

Nseq = numel(Yseq);

k_new = k+1;
% reset if we've been round the sequence
if k_new>Nseq
    k_new=1;
end

yD_k = Yseq(k);
u_k = Useq(k);

e_k = yD_k - y_k;

% 复制控制序列
Useq_new = Useq;

% 边界处理
k_learn = k-1;
if k_learn<1
    k_learn=Nseq;
end

% ILC更新律
Useq_new(k_learn) = Useq_new(k_learn) + gamma*e_k;
