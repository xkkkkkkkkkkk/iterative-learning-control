function Nodevector = U_quasi_uniform(n,k)
% 准均匀B样条的节点向量计算，共n+1个控制几点，k次B样条，k+1阶 
Nodevector = zeros(1,n+k+2);
piecevise = n - k + 1;      % 曲线的段数
if piecevise == 1           % 只有一段曲线时，n=k
    for i = k+2 : n+k+2
        Nodevector(1,i) = 1;
    end
else
    flag = 1;               % 不止一段曲线时
    while flag ~= piecevise
        Nodevector(1,k+flag+1) = Nodevector(1,k+flag) + 1/piecevise;
        flag = flag + 1;
    end
    Nodevector(1,n+2 : n+k+2) = 1;  % 节点向量前面和后面有（k+1）个重复值（阶数）
end