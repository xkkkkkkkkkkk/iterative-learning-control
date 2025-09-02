clc
clear
close all

%% 数据定义
d = 3.5;
P = [0, 10, 25, 25, 40, 50;
     -d/2, -d/2, -d/2+0.5, d/2-0.5, d/2, d/2];           % 6个控制点，满足曲率连续
n = size(P,2)-1;                                     % n:控制点个数，从0开始计数
k = 4;                                               % k阶，k-1次B样条
flag = 2;                                            % 1，2分别为均匀B样条，准均匀B样条


%% 生成B样条曲线

path = [];
Bik = zeros(n+1,1);

if flag == 1       % 均匀B样条
    Nodevector = linspace(0,1,n+k+1); % 节点矢量
    for u = (k-1)/(n+k+1) : 0.001 : (n+1)/(n+k+1)
        for i = 0 : 1 : n
            Bik(i+1, 1) = Basefunction(i, k-1, u, Nodevector);
        end
        p_u = P * Bik;
        path = [path: [p_u(1,1),p_u(2,1)]];
    end
elseif flag == 2
    Nodevector = U_quasi_uniform(n,k-1); % 准均匀B样条节点矢量
    for u = 0 : 0.005 : 1-0.005
        for i = 0 : 1 : n
            Bik(i+1, 1) = Basefunction(i, k-1, u, Nodevector);
        end
    end
    p_u = P * Bik;
    path = [path:[p_u(1),p_u(2)]];
else
    fprintf('error:\n')
end

