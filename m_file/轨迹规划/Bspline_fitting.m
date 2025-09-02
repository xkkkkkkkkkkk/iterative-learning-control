%% 筛选型值点

% 参数设置
theta_circle = 40;
epsilon_allow_max = 0.02;
break_points = []; % 断点索引存储

%% 转角断点
for i  = 2: length(Q_real) - 1;
    vec1 = Q_real(i-1,:) - Q_real(i,:);
    vec2 = Q_real(i+1,:) - Q_real(i,:);
    cos = dot(vec1,vec2) / (norm(vec1)*norm(vec2));
    theta_i = acosd(cos); % 转换为角度

    if theta_i > theta_circle
        break_points = [break_points, i];
    end
end

%% 双边弓高误差

% 此时是将i两侧的角近似相等了，与之对应的左右弓高误差用的也都是一个角
for i = 2: length(Q_real)-1
    l_i = norm(Q_real(i-1,:)-Q_real(i,:));
    l_i1 = norm(Q_real(i,:)-Q_real(i+1,:));
    r = (l_i + l_i1) / (4*sind(theta_i/2)); 

    delta_i = r * (1 - cosd(theta_i/2));
    delta_i1 = r * (1 - cosd(theta_i/2));
    epsilon_max = max(delta_i1,delta_i);

    if epsilon_max > epsilon_allow_max
        break_points = [break_points, i];
    end
end

%% 凹凸性检测

for i = 2: length(Q_real)-1
   cross_dot = vec1(2)*vec2(1) - vec1(1)-vec2(2);
   if i > 2 && sign(cross_dot) ~= sign(last_cross_dot)
       break_points = [break_points, i];
   end
   last_cross_dot = cross_dot;
end

%% 断点去重排序
break_points = unique(sort(break_points));

%% 累计弦长法参数化

function U = chord_length(Q_segment)
% 输入：分段型值点集 （N*2矩阵）
% 输出：节点矢量U（u0,u1,u2.....uM）

    n = size(Q_segment, 1);
    L = 0;
    for i = 2:n
        L = L + norm(Q_segment(i,:) - Q_segment(i-1,:));
    end
    U = zeors(1,n);
    U(1) = 0;
    for i = 2: n-1
        seg_len = norm(Q_segment(i,:) - Q_segment(i-1,:));
        U(i) = U(i-1) + seg_len/L;
    end
    U(n) = 1;
end

%% 三次B样条控制点反算

function P = reverse_caculate(Q_segment)
% 输入：分段型值点集
% 输出：控制点
    
    m = size(Q_segment, 1)-1;
    U = chord_length(Q_segment); %节点矢量

    N = zeros(m+1, m+3);
    for i = 0:m
        for j = 0:m+2
            N(i+1,j+1) = DeBoor(j, 3, U(j+1));
        end
    end

% 边界条件：P0 = Q0，Pn = Qm

    P = zeros(m+3, 2);
    P(1,:) = Q_segment(1,:);
    P(end,:) = Q_segment(end,:);

    for dim = 1:2
        Q_vec = Q_segment(:,dim);
        A = N(2:end-1, 2:end-1);
        b = Q_vec(2:end-1) - N(2:end-1)*P(1,dim) - N(2:end-1)*P(end,dim);
        P_inner = A \ b;
        P(2:end-1, dim) = P_inner;
    end
end

% 德布尔递推公式
function N_val = DeBoor(i,k,u)
    global U_full; % 完整节点矢量
    if k == 0
        N_val = (U_full(i) <= u) && (u < U_full(i+1));
    else
        term1 = 0;
        term2 = 0;
        
        B1 = U_full(i+k) - U_full(i);
        if B1 > 0
            term1 = (u - U_full(i)) / B1 * DeBoor(i,k-1,u);
        end

        B2 = U_full(i+k+1) - U_full(i+1);
        if B2 > 0
            term2 = (U_full(i+k) - u) / B2 * DeBoor(i+1,k-1,u);
        end

        N_val = B1 + B2;
    end
end