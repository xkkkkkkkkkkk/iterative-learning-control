clear; close all; clc;

%% 参数设置
% 机械臂参数注释
% DH_params                 DH参数表
% r                    各关节质心坐标
% fb                        粘滞摩擦系数
% fc                        库仑摩擦系数
% m                         连杆质量
d1 = 0.33; d2 = 0.645; d3 = 0.115;  % d参数
L1 = 1.15; L2 = 1.22; L3 = 0.215;  % 连杆长度
DH_params=[0.33,-0.645,0,0,0;    
    1.15,0.645,0,pi/2,pi/2;
    1.22,0.115,0,0,0;
    0.215,0.17415,0,0,pi/2;
    0,0.11985,0,pi/2,0;
    0,0.11655,0,-pi/2,0];
g = [0, 0, 9.81];    
m = [372.27, 232.06, 202.83, 66.0035248, 0, 0];  
r = [
    0.0347521,  0.0079348,  -0.236716;  
    0.486870792, 0,  -0.117636803;   
    0.163764236, 0,  0.141641296;  
    0, 0,  -0.300590996;   
    0, 0,  0;     
    0, 0,  0      
    ];
% 惯性张量转换 
user_I = [9.4005009,  9.4005009, 9.4005009, 0, 0, 0;                        % 连杆1
            0, 0, 31.50357447, 0, 0, 0;                                     % 连杆2
            6.25921139178724, 58.47158264, 58.47158264, 0, 0, 0;            % 连杆3
            21.8165623219505, 3.59647086679311, 21.43499647496,0, 0, 0;     % 连杆4
            0, 0, 0,0, 0, 0;                                                % 连杆5
            0,0,0,0, 0, 0];
I = zeros(3,3,6);

for i = 1:6
    I(:,:,i) = [user_I(i,1), user_I(i,2), user_I(i,3);
               user_I(i,2), user_I(i,4), user_I(i,5);
               user_I(i,3), user_I(i,5), user_I(i,6)];
end
 
B = [599.361/(256.857^2),476.578/(269.176^2),374.719/(251.333^2),...
    56.835/(188.855^2),126.935/(190.192^2),126.084/(131.340^2)];   
Fc=[323.836/256.857,-323.836/256.857;
    632.69/269.176,-632.69/269.176;
    451.073/251.333,-451.073/251.333;
    126.608/269.176,-126.608/269.176;
    86.797/190.192,-86.797/190.192;
    28.362/131.340,-28.362/131.340;];    
% B = [0,0,0,0,0,0];
% Fc = [0,0,0,0,0,0];
Ts = 0.01;          % 采样时间 (10ms)
T = 2;              % 轨迹持续时间 (2秒)
t = 0:Ts:T;         % 时间向量
N = length(t);      % 时间步数
iter_max = 50;     % 最大迭代次数 (减少以加快仿真)
n_joints = size(DH_params, 1); % 关节数 (6)
test_fc(g, DH_params, m, r, I, B, Fc);

%% 轨迹规划 (五次多项式，每个关节独立)
q_des = zeros(n_joints, N);    % 期望位置
qd_des = zeros(n_joints, N);   % 期望速度
qdd_des = zeros(n_joints, N);  % 期望加速度

for j = 1:n_joints
    q0 = 0;                   % 起始位置
    qf = (pi/2) * (j/2);      % 结束位置
    % 五次多项式插值
    q_des(j,:) = q0 + (qf - q0) * (10*(t/T).^3 - 15*(t/T).^4 + 6*(t/T).^5);
    qd_des(j,:) = [diff(q_des(j,:)) / Ts, 0]; 
    qdd_des(j,:) = [diff(qd_des(j,:)) / Ts, 0]; 
end

%% ILC
% ILC参数注释
% iter                                      当前迭代次数
% q                                         当前关节位置
% qd_act                                    当前关节速度
% y                                         实际关节位置轨迹
% u_fb                                      反馈控制力矩
% e                                         位置误差
% edot                                      速度误差
% u_total                                   总控制力矩
% e_iter                                    当前迭代的全程误差
% joint_rmse                                各关节均方差
% converged                                 收敛标志
% e_joint                                   单个关节误差轨迹
% de                                        误差导数
% delta_u                                   力矩增量
% delta_u_filt                              滤波后力矩增量
Lp = 0.5 * ones(1, n_joints);               % P学习增益
Ld = 0.1 * ones(1, n_joints);               % D学习增益
Q = 0.2;                                    % 低通滤波器系数
alpha = 0.95;                               % 遗忘因子
Kp = 300 * ones(1, n_joints);                % 比例增益P
Kd = 30 * ones(1, n_joints);                 % 微分增益D
e_history = zeros(iter_max, N, n_joints);   % 误差历史
u_history = zeros(iter_max, n_joints, N);   % 控制信号历史
rmse = zeros(iter_max, 1);                  % 总体RMSE历史
u_ff = zeros(n_joints, N);                  % 前馈控制力矩
u_ff_model = zeros(n_joints, N);            % 理论计算的前馈控制力矩

for k = 1:N
    u_ff_model(:, k) = inverseDynamics(q_des(:, k), qd_des(:, k), qdd_des(:, k), g, DH_params, m, r, I, B, Fc);
end

model_confidence = 0.9; % 模型置信度
u_ff = model_confidence * u_ff_model;
% 主循环
for iter = 1:iter_max
    % 初始化状态
    q_act = zeros(n_joints, 1);      
    qd_act = zeros(n_joints, 1); 
    y = zeros(n_joints, N);      
    y(:,1) = q_act;                  
    u_fb = zeros(n_joints, N);   
    
    % 运行单次迭代
    for k = 1:N-1
        % 反馈控制
        e = q_des(:, k) - y(:,k);          
        edot = qd_des(:, k) - qd_act;
        u_fb(:, k) = Kp' .* e + Kd' .* edot; 
        u_total = u_ff(:, k) + u_fb(:, k);
        % 正向动力学更新状态
        [q_new, qd_new] = forwardDynamics(y(:,k), qd_act, u_total, g, DH_params, m, r, I, B, Fc, Ts);
        q_act = q_new;
        qd_act = qd_new;
        y(:, k+1) = q_act;
        e_history(iter, k, :) = e;
    end
    
    % 计算跟踪误差
    e_iter = q_des - y;
    joint_rmse = sqrt(mean(e_iter.^2, 2)); 
    rmse(iter) = mean(joint_rmse);          
    % 存储控制信号
    u_history(iter, :, :) = u_ff;
    fprintf('迭代 %3d: 总体RMSE = %.6f rad\n', iter, rmse(iter));

    if iter > 1 && abs(rmse(iter) - rmse(iter-1)) < 1e-6
        fprintf('收敛于迭代 %d\n', iter);
        break;
    end
    
    % ILC更新律 
    if iter < iter_max 
        for j = 1:n_joints
            e_joint = e_iter(j, :); 
            de = [0,diff(e_joint) / Ts];
            delta_u = Lp(j)*e_joint + Ld(j)*de;
            % 低通滤波
            delta_u_filt = filter(1-Q, [1, -Q], delta_u);
            % 更新前馈信号 (带遗忘因子)
            u_ff(j, :) = alpha*u_ff(j, :) + delta_u_filt;
        end
    end
end
% 在第一次迭代的第一个时间步输出关键数据
if iter == 1 && k == 1
     fprintf('关节1初始前馈力矩: %.2f N·m\n', u_ff(1,1));
     fprintf('关节1模型力矩: %.2f N·m\n', u_ff_model(1,1));
     fprintf('静态摩擦力矩: %.2f N·m\n', Fc(1));
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
    plot(t, q_des(j+3, :), 'r--', 'LineWidth', 2);
    plot(t, y(j, :), 'b-', 'LineWidth', 1.5);
    xlabel('时间 (s)');
    ylabel('位置 (rad)');
    legend('期望', '实际', 'Location', 'best');
    title(sprintf('关节 %d 轨迹跟踪', j+3));
    grid on;
end

% 控制信号 (关节1)
figure;
subplot(2,1,1);
plot(t, u_ff(2, :), 'LineWidth', 1.5);
xlabel('时间 (s)');
ylabel('前馈控制 (Nm)');
title('关节2前馈控制信号');
grid on;

subplot(2,1,2);
plot(t, u_fb(2, :), 'LineWidth', 1.5);
xlabel('时间 (s)');
ylabel('反馈控制 (Nm)');
title('关节2反馈控制信号');
grid on;

% 误差分布图
figure;
boxplot(squeeze(e_history(iter, :, :)));
xlabel('关节编号');
ylabel('跟踪误差 (rad)');
title('最终迭代各关节误差分布');
grid on;

function tau_g = test_fc(g, DH_params, m, r, I, fb, fc)

g = [0, 0, 9.81];

tau_g = inverseDynamics(zeros(6,1),zeros(6,1),zeros(6,1), g, DH_params, m, r, I, fb, fc);
fprintf('\n重力矩\n');
disp(tau_g);
    for i = 1:6
        if abs(tau_g(i)) < fc(i)
            fprintf('关节%.2f重力矩小于静摩擦，无法启动\n',i);
        else
            fprintf('一切正常');
        end
    end
end