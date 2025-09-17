% 六轴机械臂迭代学习控制（ILC）用于轨迹跟踪精度补偿
clear; close all; clc;

%% 1. 仿真参数设置
dt = 0.01;                     % 时间步长（s）
T = 5;                         % 总仿真时间（s）
t = 0:dt:T;                    % 时间向量
N = length(t);                 % 时间步数
n_iter = 15;                   % ILC迭代次数

%% 2. 机械臂动力学参数（自拟简化模型）
% 假设各关节动力学为惯性+阻尼模型：M*ddq + B*dq = tau
M = diag([0.5, 0.4, 0.3, 0.25, 0.2, 0.15]);  % 关节惯性矩阵（6x6）
B = diag([0.1, 0.08, 0.06, 0.05, 0.04, 0.03]); % 关节阻尼矩阵（6x6）

%% 3. 期望轨迹（六关节正弦参考轨迹）
% 各关节参考轨迹参数：幅值(A)、频率(w)、相位(phi)
A = [0.8, 0.6, 0.7, 0.5, 0.4, 0.3];          % 幅值（rad）
w = [1.5, 2.0, 1.8, 2.2, 2.5, 1.2] * pi;     % 角频率（rad/s）
phi = [0, pi/4, pi/2, 3*pi/4, pi, 5*pi/4];   % 相位（rad）

% 生成参考轨迹（位置、速度、加速度）
q_ref = zeros(6, N);
dq_ref = zeros(6, N);
ddq_ref = zeros(6, N);
for j = 1:6
    q_ref(j,:) = A(j) * sin(w(j)*t + phi(j));
    dq_ref(j,:) = A(j)*w(j) * cos(w(j)*t + phi(j));
    ddq_ref(j,:) = -A(j)*w(j)^2 * sin(w(j)*t + phi(j));
end

%% 4. ILC参数初始化
Gamma = 0.6 * eye(6);         % 学习增益矩阵（6x6对角阵）
tau_ff = zeros(6, N);          % 前馈控制力矩（初始为零）
K_p = diag([12, 10, 8, 7, 6, 5]);   % 反馈比例增益矩阵
K_d = diag([3, 2.5, 2, 1.5, 1.2, 1]); % 反馈微分增益矩阵

% 存储每次迭代的误差和实际轨迹
q_actual = zeros(6, N, n_iter); % 各次迭代的实际关节位置
errors = zeros(n_iter, 6);      % 各次迭代各关节的RMSE

%% 5. ILC主循环
for iter = 1:n_iter
    % 初始化状态
    q = zeros(6,1);            % 初始关节位置
    dq = zeros(6,1);           % 初始关节速度
    
    % 当前迭代的实际轨迹存储
    q_iter = zeros(6, N);
    
    % 正向仿真（使用当前前馈力矩tau_ff）
    for k = 1:N-1
        % 跟踪误差计算
        e = q_ref(:,k) - q;
        de = dq_ref(:,k) - dq;
        
        % 控制力矩计算：前馈 + 反馈（PD）
        tau_fb = K_p * e + K_d * de;
        tau_total = tau_ff(:,k) + tau_fb;
        
        % 动力学方程计算加速度：M*ddq + B*dq = tau
        ddq = M \ (tau_total - B*dq);
        
        % 状态更新（欧拉积分）
        dq = dq + ddq * dt;
        q = q + dq * dt;
        
        % 存储当前时刻关节位置
        q_iter(:,k) = q;
    end
    q_iter(:,N) = q; % 存储最终时刻位置
    
    % 记录本次迭代的实际轨迹
    q_actual(:,:,iter) = q_iter;
    
    % 计算各关节RMSE
    for j = 1:6
        errors(iter,j) = sqrt(mean((q_ref(j,:) - q_iter(j,:)).^2));
    end
    
    % ILC更新律：更新前馈力矩（P型学习律）
    if iter < n_iter
        tau_ff = tau_ff + Gamma * (q_ref - q_iter);
    end
    
    fprintf('迭代 %d 完成，平均RMSE: %.4f rad\n', iter, mean(errors(iter,:)));
end

%% 6. 结果可视化
% 绘制各关节最后一次迭代的跟踪效果
figure('Position', [100, 100, 1200, 800]);
for j = 1:6
    subplot(3,2,j);
    plot(t, q_ref(j,:), 'r-', 'LineWidth', 2); hold on;
    plot(t, squeeze(q_actual(j,:,n_iter)), 'b--', 'LineWidth', 1.5);
    title(sprintf('关节%d轨迹跟踪', j));
    xlabel('时间 (s)'); ylabel('角度 (rad)');
    legend('期望轨迹', '实际轨迹');
    grid on;
end

% 绘制各关节误差收敛曲线
figure('Position', [100, 100, 1200, 600]);
for j = 1:6
    subplot(2,3,j);
    plot(1:n_iter, errors(:,j), 'o-', 'LineWidth', 1.5);
    title(sprintf('关节%d误差收敛', j));
    xlabel('迭代次数'); ylabel('RMSE (rad)');
    grid on;
end

% 显示最终迭代的误差统计
fprintf('\n最终迭代各关节RMSE（rad）:\n');
disp(errors(n_iter,:));
fprintf('平均RMSE: %.4f rad\n', mean(errors(n_iter,:)));