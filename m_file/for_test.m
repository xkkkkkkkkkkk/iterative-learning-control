clear; close all; clc;

%% 机械臂参数设置（用户需根据实际机器人修改）
% DH参数 [a, alpha, d, theta] - 标准DH表
% 假设后三个关节的d=0，alpha根据常见配置设定
% 注意：theta将在轨迹规划中设置，这里初始化为0
d1 = 0.33; d2 = 0.645; d3 = 0.115;  % 用户提供的d参数
L1 = 1.15; L2 = 1.22; L3 = 0.215;  % 用户提供的连杆长度
DH_params = [
    0,   pi/2, d1, 0;   % 关节1
    L1,  0,    d2, 0;   % 关节2
    L2,  0,    d3, 0;   % 关节3
    L3,  pi/2, 0,  0;   % 关节4
    0,  -pi/2, 0,  0;   % 关节5
    0,   0,    0,  0    % 关节6
];

% 动力学参数
g = 9.81;  % 重力加速度 (m/s^2)

% 质量 (kg) 
m = [372.27, 232.06, 202.83, 66.0035248, 0, 0]; 

% 质心位置 (在各自连杆坐标系中) - 用户提供
cm_pos = [
    0.0347521,  0.0079348,  -0.236716;  % 关节1质心
    0.486870792, 0,  -0.117636803;   % 关节2质心
    0.163764236, 0,  0.141641296;   % 关节3质心
    0, 0,  -0.300590996;   % 关节4质心
    0, 0,  0;     % 关节5质心
    0, 0,  0      % 关节6质心
];

user_I = [9.4005009,  9.4005009, 9.4005009, 0, 0, 0;                      % 连杆1
            0, 0, 31.50357447, 0, 0, 0;                                     % 连杆2
            6.25921139178724, 58.47158264, 58.47158264, 0, 0, 0;            % 连杆3
            21.8165623219505, 3.59647086679311, 21.43499647496,0, 0, 0;     % 连杆4
            0, 0, 0,0, 0, 0;                                                % 连杆5
            0,0,0,0, 0, 0];
% 惯性张量转换 (在质心坐标系中) 

% 格式: {I1, I2, ..., I6} 每个为3x3矩阵
I = zeros(3,3,6);
for i = 1:6
    I(:,:,i) = [user_I(i,1), user_I(i,2), user_I(i,3);
               user_I(i,2), user_I(i,4), user_I(i,5);
               user_I(i,3), user_I(i,5), user_I(i,6)];
end

% 摩擦参数 
fb = [599.361145, 476.578278, 374.719635, 56.8351135, 126.935608, 126.084808];    % 粘滞摩擦系数 (N·m·s/rad)
fc = [323.836395, 632.690613, 451.073, 126.608192, 86.796814, 28.3622684];   % 库仑摩擦系数 (N·m)

%% 仿真参数
Ts = 0.01;          % 采样时间 (10ms)
T = 2;              % 轨迹持续时间 (2秒)
t = 0:Ts:T;         % 时间向量
N = length(t);      % 时间步数
iter_max = 50;      % 最大迭代次数 (减少以加快仿真)
n_joints = size(DH_params, 1); % 关节数 (6)

%% 轨迹规划 (五次多项式，每个关节独立)
qd = zeros(n_joints, N);       % 期望位置
qd_dot = zeros(n_joints, N);   % 期望速度
qd_ddot = zeros(n_joints, N);  % 期望加速度

for j = 1:n_joints
    qd0 = 0; % 起始位置
    qdf = (pi/3) * (j/3); % 结束位置 (不同关节不同)
    
    % 五次多项式插值
    qd(j,:) = qd0 + (qdf - qd0) * (10*(t/T).^3 - 15*(t/T).^4 + 6*(t/T).^5);
    qd_dot(j,:) = [diff(qd(j,:)) / Ts, 0]; 
    qd_ddot(j,:) = [diff(qd_dot(j,:)) / Ts, 0];
end

%% ILC参数 (每个关节独立)
Lp = 0.8 * ones(1, n_joints);    % P学习增益
Ld = 0.2 * ones(1, n_joints);    % D学习增益
Q = 0.1;                         % 低通滤波器系数
alpha = 0.99;                    % 遗忘因子

% 反馈控制参数 (PD)
Kp = 100 * ones(1, n_joints);    % 比例增益
Kd = 10 * ones(1, n_joints);     % 微分增益

%% 初始化存储变量
u_ff = zeros(n_joints, N);       % 前馈控制信号
e_history = zeros(iter_max, N, n_joints); % 误差历史
u_history = zeros(iter_max, n_joints, N); % 控制信号历史
rmse = zeros(iter_max, 1);               % 总体RMSE历史

%% 机械臂逆动力学函数 (牛顿-欧拉算法)
function tau = inverseDynamics(q, qd, qdd, g, DH_params, m, cm_pos, I, b, fc)
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
        ct = cos(theta(i)); st = sin(theta(i));
        ca = cos(alpha(i)); sa = sin(alpha(i));
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
        
        % 质心线加速度
        r_cm = cm_pos(i,:)'; % 质心位置向量
        dv_cm = dv(:,i+1) + cross(dw(:,i+1), r_cm) + cross(w(:,i+1), cross(w(:,i+1), r_cm));
    end
    
    % 后向传递 (从末端到基座)
    f = zeros(3, n+1);    % 力
    n_t = zeros(3, n+1);  % 力矩
    
    for i = n:-1:1
        % 质心动力学
        F = m(i) * dv_cm;
        N = I(:,:,i) * dw(:,i+1) + cross(w(:,i+1), I(:,:,i)*w(:,i+1));
        
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
        P_next = [a(i); d(i)*sa; d(i)*ca]; % i坐标系中的位置向量
        
        if i == n
            f(:,i) = F;
            n_t(:,i) = N + cross(r_cm, F);
        else
            f(:,i) = R * f(:,i+1) + F;
            n_t(:,i) = R * n_t(:,i+1) + cross(r_cm, F) + cross(P_next, R*f(:,i+1)) + N;
        end
        
        % 关节力矩 (考虑摩擦)
        tau_friction = b(i)*qd(i) + fc(i)*tanh(100*qd(i)); % 使用tanh近似sign函数
        tau(i) = n_t(3,i) + tau_friction;
    end
end

%% 正向动力学函数 (欧拉积分)
function [q_new, qd_new] = forwardDynamics(q, qd, tau, g, DH_params, m, cm_pos, I, fb, fc, Ts)
    % 计算当前状态下的加速度
    qdd = zeros(size(q));
    
    % 使用逆动力学计算所需力矩 (验证用)
    tau_calc = inverseDynamics(q, qd, qdd, g, DH_params, m, cm_pos, I, fb, fc);
    
    % 简化正向动力学 (实际系统响应)
    for j = 1:length(q)
        % 有效惯量 (包含电机转子惯量)
        J_eff = 0.1 + 0.05*m(j);
        
        % 计算加速度 (考虑摩擦)
        tau_friction = fb(j)*qd(j) + fc(j)*tanh(100*qd(j));
        qdd(j) = (tau(j) - tau_friction) / J_eff;
    end
    
    % 状态更新 (欧拉积分)
    qd_new = qd + qdd * Ts;
    q_new = q + qd * Ts + 0.5 * qdd * Ts^2;
end

%% ILC主循环
for iter = 1:iter_max
    % 初始化状态
    q = zeros(n_joints, 1);      % 关节位置
    qd_act = zeros(n_joints, 1); % 关节速度
    y = zeros(n_joints, N);      % 实际位置
    y(:,1) = q;                  % 初始位置
    u_fb = zeros(n_joints, N);   % 反馈控制
    
    % 运行单次迭代
    for k = 1:N-1
        % 反馈控制
        e = qd(:, k) - q;          % 位置误差
        edot = qd_dot(:, k) - qd_act; % 速度误差
        u_fb(:, k) = Kp' .* e + Kd' .* edot; % PD反馈
        
        % 总控制量 = 前馈 + 反馈
        u_total = u_ff(:, k) + u_fb(:, k);
        
        % 使用正向动力学更新状态
        [q, qd_act] = forwardDynamics(q, qd_act, u_total, g, DH_params, m, cm_pos, I, fb, fc, Ts);
        
        % 存储实际位置
        y(:, k+1) = q;
        
        % 存储误差
        e_history(iter, k, :) = e;
    end
    
    % 计算本次迭代的跟踪误差
    e_iter = qd - y;
    joint_rmse = sqrt(mean(e_iter.^2, 2)); % 每个关节的RMSE
    rmse(iter) = mean(joint_rmse);         % 总体RMSE
    
    % 存储控制信号
    u_history(iter, :, :) = u_ff;
    
    % 显示进度
    fprintf('迭代 %3d: 总体RMSE = %.6f rad\n', iter, rmse(iter));
    
    % 停止条件检查
    if iter > 1 && abs(rmse(iter) - rmse(iter-1)) < 1e-6
        fprintf('收敛于迭代 %d\n', iter);
        break;
    end
    
    % ILC更新律 (PD型 + 低通滤波)
    if iter < iter_max
        for j = 1:n_joints
            % 提取当前关节误差
            e_joint = squeeze(e_iter(j, :));
            
            % 计算误差导数
            de = diff(e_joint) / Ts;
            de = [de, de(end)];
            
            % PD型学习律
            delta_u = Lp(j)*e_joint + Ld(j)*de;
            
            % 应用低通滤波
            delta_u_filt = filter(1-Q, [1, -Q], delta_u);
            
            % 更新前馈信号 (带遗忘因子)
            u_ff(j, :) = alpha*u_ff(j, :) + delta_u_filt;
        end
    end
end

%% 结果可视化
% 迭代收敛曲线
figure;
semilogy(1:iter, rmse(1:iter), 'o-', 'LineWidth', 1.5, 'MarkerSize', 8);
xlabel('迭代次数');
ylabel('总体RMSE (rad)');
title('ILC收敛过程');
grid on;

% 轨迹跟踪结果 (关节1-3)
figure;
for j = 1:3
    subplot(3,1,j);
    hold on;
    plot(t, qd(j, :), 'r--', 'LineWidth', 2);
    plot(t, y(j, :), 'b-', 'LineWidth', 1.5);
    xlabel('时间 (s)');
    ylabel('位置 (rad)');
    legend('期望', '实际', 'Location', 'best');
    title(sprintf('关节 %d 轨迹跟踪', j));
    grid on;
end

% 控制信号 (关节1)
figure;
subplot(2,1,1);
plot(t, u_ff(1, :), 'LineWidth', 1.5);
xlabel('时间 (s)');
ylabel('前馈控制 (Nm)');
title('关节1前馈控制信号');
grid on;

subplot(2,1,2);
plot(t, u_fb(1, :), 'LineWidth', 1.5);
xlabel('时间 (s)');
ylabel('反馈控制 (Nm)');
title('关节1反馈控制信号');
grid on;

% 误差分布图
figure;
boxplot(squeeze(e_history(iter, :, :)));
xlabel('关节编号');
ylabel('跟踪误差 (rad)');
title('最终迭代各关节误差分布');
grid on;