clear; 
close all; 
clc;

%% 参数设置
Ts = 0.01;          % 采样时间 (10ms)
T = 50;              % 轨迹持续时间 (2秒)
t = 0:Ts:T;         % 时间向量
N = length(t);      % 时间步数
iter_max =  1000;      % 最大迭代次数

% 机械臂模型参数 (二阶系统近似)
m = 1.5;            % 等效质量 (kg)
b = 2.0;            % 等效阻尼 (N·s/m)
k = 10.0;           % 等效刚度 (N/m)

% 状态空间模型 (连续时间)
A = [0 1; -k/m -b/m];
B = [0; 1/m];
C = [1 0];
D = 0;

sys_c = ss(A, B, C, D);

% 离散化
sys_d = c2d(sys_c, Ts, 'zoh');
[Ad, Bd, Cd, Dd] = ssdata(sys_d);

% ILC参数
Lp = 0.8;           % P学习增益
Ld = 0.2;           % D学习增益
Q = 0.1;            % 低通滤波器Q (0-1之间)
alpha = 0.99;       % 遗忘因子

%% 期望轨迹 (五次多项式)
t0 = 0; tf = T;
qd0 = 0; qdf = 1;   % 起始和结束位置
qd = qd0 + (qdf - qd0)*(10*(t/tf).^3 - 15*(t/tf).^4 + 6*(t/tf).^5);
qd_dot = diff(qd)/Ts; qd_dot = [qd_dot, qd_dot(end)]; % 一阶导数
qd_ddot = diff(qd_dot)/Ts; qd_ddot = [qd_ddot, qd_ddot(end)]; % 二阶导数

%% 初始化存储变量
u_ff = zeros(1, N);     % 前馈控制信号
e_history = zeros(iter_max, N); % 误差历史
u_history = zeros(iter_max, N); % 控制信号历史
rmse = zeros(iter_max, 1);      % RMSE历史

%% 迭代学习控制主循环
for iter = 1:iter_max
    % 初始化状态
    x = [0; 0]; % 位置和速度
    y = zeros(1, N);
    u_fb = zeros(1, N);
    
    % 反馈控制器参数 (PD控制)
    Kp = 50;
    Kd = 5;
    
    % 运行单次迭代
    for k = 1:N-1
        % 反馈控制
        e = qd(k) - y(k); % 位置差 y：位置
        edot = qd_dot(k) - x(2);
        u_fb(k) = Kp*e + Kd*edot; % 反馈控制
        
        % 总控制量 = 前馈 + 反馈
        u_total = u_ff(k) + u_fb(k);
        
        % 系统动态更新
        x = Ad*x + Bd*u_total;
        y(k+1) = Cd*x;
        
        % 存储误差
        e_history(iter, k) = e;
    end
    
    % 计算本次迭代的跟踪误差
    e_iter = qd - y;
    rmse(iter) = sqrt(mean(e_iter.^2));
    
    % 存储控制信号
    u_history(iter, :) = u_ff;
    
    % 显示进度
    fprintf('迭代 %2d: RMSE = %.6f rad\n', iter, rmse(iter));
    
    % 停止条件检查
    if iter > 1 && abs(rmse(iter) - rmse(iter-1)) < 1e-6
        fprintf('收敛于迭代 %d\n', iter);
        break;
    end
    
    % ILC更新律 (PD型 + 低通滤波)
    if iter < iter_max
        % 计算误差导数
        de = diff(e_iter)/Ts;
        de = [de, de(end)];
        
        % PD型学习律
        delta_u = Lp*e_iter + Ld*de;
        
        % 应用低通滤波
        delta_u_filt = filter(1-Q, 1 -Q, delta_u);
        
        % 更新前馈信号 (带遗忘因子)
        u_ff = alpha*u_ff + delta_u_filt;
    end
end

%% 结果可视化
% 跟踪误差曲线
% figure;
% plot(t, e_iter, 'LineWidth', 1.5);
% xlabel('时间 (s)');
% ylabel('跟踪误差 (rad)');
% title(sprintf('最终迭代跟踪误差 (RMSE=%.4f rad)', rmse(iter)));
% grid on;

% 迭代过程收敛曲线
figure;
semilogy(1:iter, rmse(1:iter), 'o-', 'LineWidth', 1.5, 'MarkerSize', 8);
xlabel('迭代次数');
ylabel('RMSE (rad)');
title('迭代学习收敛过程');
grid on;

% 期望轨迹与实际轨迹对比
figure;
hold on;
plot(t, qd, 'r--', 'LineWidth', 2);
plot(t, y, 'b-', 'LineWidth', 1.5);
xlabel('时间 (s)');
ylabel('位置 (rad)');
legend('期望轨迹', '实际轨迹', 'Location', 'best');
title('轨迹跟踪性能');
grid on;

% 控制信号可视化
% figure;
% subplot(2,1,1);
% plot(t, u_ff, 'LineWidth', 1.5);
% xlabel('时间 (s)');
% ylabel('前馈控制 (Nm)');
% title('学习得到的前馈控制信号');
% grid on;
% 
% subplot(2,1,2);
% plot(t, u_fb, 'LineWidth', 1.5);
% xlabel('时间 (s)');
% ylabel('反馈控制 (Nm)');
% title('反馈控制信号');
% grid on;
inverseDynamics
% 学习过程动画 (前几次迭代)
% figure;
% for i = 1:min(4, iter)
%     subplot(2,2,i);
%     plot(t, qd, 'r--', 'LineWidth', 2);
%     hold on;
%     plot(t, e_history(i,:) + qd, 'b-', 'LineWidth', 1.5);
%     title(sprintf('迭代 %d: RMSE=%.4f', i, rmse(i)));
%     xlabel('时间 (s)');
%     ylabel('位置 (rad)');
%     legend('期望', '实际', 'Location', 'best');
%     grid on;
%     axis([0 T -0.2 1.2]);
%  end