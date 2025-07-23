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

% 惯性张量 (在质心坐标系中) - 用户提供
% 格式: {I1, I2, ..., I6} 每个为3x3矩阵
I = cell(1,6);
for i = 1:6
    % 简化为对角矩阵，实际需用户提供完整张量
    I{i} = diag([0.1, 0.1, 0.1]) * m(i);
end

% 摩擦参数 - 用户提供
b = [599.361145, 476.578278, 374.719635, 56.8351135, 126.935608, 126.084808];    % 粘滞摩擦系数 (N·m·s/rad)
fc = [323.836395, 632.690613, 451.073, 126.608192, 86.796814, 28.3622684];   % 库仑摩擦系数 (N·m)

%% 仿真参数
Ts = 0.01;          % 采样时间 (10ms)
T = 2;              % 轨迹持续时间 (2秒)
t = 0:Ts:T;         % 时间向量
N = length(t);      % 时间步数
iter_max = 100;     % 最大迭代次数 (减少以加快仿真)
n_joints = size(DH_params, 1); % 关节数 (6)

%% 轨迹规划 (五次多项式，每个关节独立)
qd = zeros(n_joints, N);       % 期望位置
qd_dot = zeros(n_joints, N);   % 期望速度
qd_ddot = zeros(n_joints, N);  % 期望加速度

for j = 1:n_joints
    qd0 = 0; % 起始位置
    qdf = (pi/4) * (j/2); % 结束位置 (不同关节不同)
    
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
Kp = 50 * ones(1, n_joints);     % 比例增益
Kd = 5 * ones(1, n_joints);      % 微分增益

%% 初始化存储变量
u_ff = zeros(n_joints, N);       % 前馈控制信号
e_history = zeros(iter_max, N, n_joints); % 误差历史
u_history = zeros(iter_max, N, n_joints); % 控制信号历史
rmse = zeros(iter_max, 1);               % 总体RMSE历史

%% 机械臂动力学函数 (牛顿-欧拉递归算法)
% 函数: 计算逆动力学 (关节力矩)
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
    dv(:,1) = [0;0;g];    % 基座加速度包含重力
    
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
        w(:,i+1) = R' * w(:,i) + [0; 0; qd(i)];
        dw(:,i+1) = R' * dw(:,i) + cross(R'*w(:,i), [0;0;qd(i)]) + [0;0;qdd(i)];
        
        % 坐标系i原点的线加速度
        P_prev = [a(i); -d(i)*sa; d(i)*ca]; % i-1坐标系中的位置向量
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
        N = I{i} * dw(:,i+1) + cross(w(:,i+1), I{i}*w(:,i+1));
        
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
        P_next = [a(i); -d(i)*sa; d(i)*ca]; % i坐标系中的位置向量
        
        if i == n
            f(:,i) = F;
            n_t(:,i) = N + cross(r_cm, F);
        else
            f(:,i) = R * f(:,i+1) + F;
            n_t(:,i) = R * n_t(:,i+1) + cross(r_cm, F) + cross(P_next, R*f(:,i+1)) + N;
        end
        
        % 关节力矩 (考虑摩擦)
        tau(i) = n_t(3,i) + b(i)*qd(i) + fc(i)*sign(qd(i));
    end
end

%% ILC主循环
for iter = 1:iter_max
    % 初始化状态
    q = zeros(n_joints, 1);    % 关节位置
    qd_act = zeros(n_joints, 1); % 关节速度
    y = zeros(n_joints, N);     % 实际位置
    u_fb = zeros(n_joints, N);  % 反馈控制
    
    % 运行单次迭代
    for k = 1:N-1
        % 反馈控制
        e = qd(:, k) - q;          % 位置误差
        edot = qd_dot(:, k) - qd_act; % 速度误差
        u_fb(:, k) = Kp' .* e + Kd' .* edot; % PD反馈
        
        % 总控制量 = 前馈 + 反馈
        u_total = u_ff(:, k) + u_fb(:, k);
        
        % 计算加速度 (使用动力学模型)
        qdd_act = zeros(n_joints, 1); % 初始化加速度
        
        % 简化的正向动力学计算 (实际应用需完整实现)
        % 注意: 这里为简化使用近似，完整实现需质量矩阵求逆
        for j = 1:n_joints
            qdd_act(j) = u_total(j) / (m(j) * 0.1); % 简化模型
        end
        
        % 状态更新 (欧拉积分)
        qd_act = qd_act + qdd_act * Ts;
        q = q + qd_act * Ts;
        
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

% 轨迹跟踪结果 (关节1示例)
jnt_plot = 1; % 选择要显示的关节
figure;
hold on;
plot(t, qd(jnt_plot, :), 'r--', 'LineWidth', 2);
plot(t, y(jnt_plot, :), 'b-', 'LineWidth', 1.5);
xlabel('时间 (s)');
ylabel('位置 (rad)');
legend('期望轨迹', '实际轨迹', 'Location', 'best');
title(sprintf('关节%d轨迹跟踪', jnt_plot));
grid on;

% 控制信号 (关节1示例)
figure;
subplot(2,1,1);
plot(t, u_ff(jnt_plot, :), 'LineWidth', 1.5);
xlabel('时间 (s)');
ylabel('前馈控制 (Nm)');
title(sprintf('关节%d前馈控制信号', jnt_plot));
grid on;

subplot(2,1,2);
plot(t, u_fb(jnt_plot, :), 'LineWidth', 1.5);
xlabel('时间 (s)');
ylabel('反馈控制 (Nm)');
title(sprintf('关节%d反馈控制信号', jnt_plot));
grid on;