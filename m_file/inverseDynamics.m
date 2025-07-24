function tau = inverseDynamics(q, qd, qdd, g, DH_params, m, cm_pos, I, fb, fc)

% 逆动力学参数注释
% q                  关节位置
% qd                 关节速度
% qdd                关节加速度
% tau                关节力矩
% w                  角速度
% dw                 角加速度
% dv                 线加速度
% f                  连杆受力
% n_t                连杆受力矩
% dv_cm              质心线加速度
% R                  旋转矩阵
% P_prev,P_next      位置向量
% r_cm               质心位置向量
% tau_friction       摩擦力矩
    n = length(q);
    tau = zeros(n, 1);
    
    % 提取DH参数
    a = DH_params(:, 1);
    alpha = DH_params(:, 2);
    d = DH_params(:, 3);
    theta = q;  % 关节变量为当前关节角度
    
    % 初始化运动学量
    w = zeros(3, n+1);    % 角速度
    dw = zeros(3, n+1);   % 角加速度
    dv = zeros(3, n+1);   % 线加速度
    w(:,1) = [0;0;0];
    dw(:,1) = [0;0;0];
    dv(:,1) = g;          % 基座加速度包含重力
    
    % 前向传递 (从基座到末端)
    for i = 1:n
        % 旋转矩阵 (i-1 到 i)
        ct = cos(theta(i)); 
        st = sin(theta(i));
        ca = cos(alpha(i)); 
        sa = sin(alpha(i));
        R = [
            ct, -st*ca,  st*sa;
            st,  ct*ca, -ct*sa;
            0,     sa,     ca
        ];
        
        % 计算角速度和角加速度
        w_i = R' * w(:,i);
        w(:,i+1) = w_i + [0; 0; qd(i)];
        dw_i = R' * dw(:,i);
        dw(:,i+1) = dw_i + cross(w_i, [0;0;qd(i)]) + [0;0;qdd(i)];
        
        % 坐标系i原点的线加速度
        P_prev = [a(i); d(i)*sa; d(i)*ca]; % i-1坐标系中的位置向量
        dv_prev_term = dv(:,i) + cross(dw(:,i), P_prev) + cross(w(:,i), cross(w(:,i), P_prev));
        dv(:,i+1) = R' * dv_prev_term;
    end
    
    % 后向传递
    f = zeros(3, n+1);    % 力
    n_t = zeros(3, n+1);  % 力矩
    dv_cm = zeros(3,n);
    
    for i = n:-1:1
        % 旋转矩阵 (i 到 i+1)
        ct = cos(theta(i)); st = sin(theta(i));
        ca = cos(alpha(i)); sa = sin(alpha(i));
        R = [
            ct, -st*ca,  st*sa;
            st,  ct*ca, -ct*sa;
            0,     sa,     ca
        ];
        
        % 坐标系i原点处的力和力矩
        r_cm = cm_pos(i,:)'; % 质心位置向量
        dv_cm_i = dv(:,i+1) + cross(dw(:,i+1), r_cm) + cross(w(:,i+1), cross(w(:,i+1), r_cm));
        F = m(i) * dv_cm_i;
        N = I(:,:,i) * dw(:,i+1) + cross(w(:,i+1), I(:,:,i)*w(:,i+1));
        P_next = [a(i); d(i)*sa; d(i)*ca]; % i坐标系中的位置向量
        
        if i == n
            f(:,i) = F;
            n_t(:,i) = N + cross(r_cm, F);
        else
            f(:,i) = R * f(:,i+1) + F;
            n_t(:,i) = R * n_t(:,i+1) + cross(r_cm, F) + cross(P_next, R*f(:,i+1)) + N;
        end
        
        % 关节力矩 (考虑摩擦)
        epsilon = 0.1;
        sign_qd = qd(i)/(abs(qd(i) + epsilon));
        tau_friction = fb(i)*qd(i) + fc(i)*sign_qd; % 使用tanh近似sign函数
        tau(i) = n_t(3,i) + tau_friction;
    end
end