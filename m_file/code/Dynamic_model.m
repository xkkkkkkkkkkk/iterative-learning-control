%% 动力学模型

% 惯性矩阵M0
M_diag = [3.5, 2.2, 1.8, 0.6, 0.4, 0.1];
M0 = diag(M_diag);
M0(1,2) = 0.8;  M0(2,1) = M0(1,2);
M0(2,3) = 0.5;  M0(3,2) = M0(2,3);
M0(3,4) = 0.2;  M0(4,3) = M0(3,4);
M0(4,5) = 0.1;  M0(5,4) = M0(4,5);

% 科里奥利力和向心力矩阵
C0 = zeros(n,n);
for j = 1:n 
    for k = 1:n 
        for i = 1:n 
            c_ijk = 0.5 * (diff(M0(k,j), q(i)) + diff(M0(k,i), q(j)) - diff(M0(i,j), q(k)));
            C(j,k) = C(j,k) + c_ijk * dq(i);
        end
    end
end
C_func = matlabFunction(C0, 'Vars', {q,dq });
C = C_func(q,dq );

% 重力项
g0 = [95; 60; 40; 15; 8; 2]; % 假设的最大重力矩
G(q) = [g0(1)];
