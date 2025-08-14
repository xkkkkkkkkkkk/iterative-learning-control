%% 完整动力学参数
T_total = 2;
Ts = 0.01;
t = 0:Ts:T_total;
N = length(t);        % 时间步数
% 标准DH参数表 (a, alpha, d, theta)
DH_params = [
    0       pi/2    0.33    0;     % A1
    1.15    0       0.645   0;     % A2
    1.22    0       0.115   0;     % A3
    0.215  -pi/2    0.33    0;     % A4
    0       pi/2    0       0;     % A5
    0      -pi/2    0.21    0      % A6
];
n_joints = size(DH_params,1);
robot_params = struct();
robot_params.DH = DH_params;
robot_params.g = [0, 0, 9.81];  % 重力向量 (z向下)
robot_params.m = [225.96, 372.27, 232.06, 202.83, 66.00, 0]'; % 各连杆质量
robot_params.cm_pos = [    % 各连杆质心位置(在连杆坐标系中)
    0, 0, 0.3225;        % Base
    0.03475, 0.00793, -0.2367; % Column
    0.4869, 0, -0.1176;  % Link
    0.1638, -0.115, 0.1416; % Arm
    0, 0, -0.3006;       % Hand
    0, 0, 0              % Flange
];
robot_params.I = getInertiaTensor(); % 3x3x6惯量张量
robot_params.gear_ratio = [-1798/7, 1321/7, -1287/7, -621/7, -621/7, -481/7];% 减速比
robot_params.motor_inertia = [0.00469, 0.0066, 0.0066, 0.0014, 0.0014, 0.0014];
robot_params.fc = [323.84, 632.69, 451.07, 126.61, 86.80, 28.36]; % 库仑摩擦
robot_params.fb = [599.36, 476.58, 374.72, 56.84, 126.94, 126.08]; % 粘滞摩擦
robot_params.fs = 1.2 * robot_params.fc;   % 静态摩擦力系数(约为库仑摩擦的1.2倍)
robot_params.vs = [0.1, 0.09, 0.08, 0.07, 0.06, 0.05]; % Stribeck速度阈值(rad/s)
robot_params.joint_limits = [     % 关节限位(弧度)
    -pi, pi;                    % A1
    deg2rad(-140), deg2rad(-5); % A2
    deg2rad(-120), deg2rad(168);% A3
    -pi, pi;                    % A4
    -pi, pi;                    % A5
    -pi, pi                     % A6
];
robot_params.max_velocity = [120, 120, 180, 180, 220, 220]; % 度/秒
robot_params.stribeck_speed = [0.08, 0.07, 0.06, 0.04, 0.04, 0.03];
robot_params.joint_type = ['R','R','R','R','R','R'];



%% 辅助函数: 获取完整的惯量张量
function I = getInertiaTensor()
    % KR210各连杆的惯量张量 (kg·m²)
    I = zeros(3,3,6);
    
    % Base (简化模型)
    I(:,:,1) = diag([9.4, 9.4, 9.4]);
    
    % Column (实际数据)
    I(3,3,2) = 31.5036;
    
    % Link
    I(:,:,3) = diag([6.2592, 58.4716, 57.2575]);
    
    % Arm
    I(:,:,4) = diag([21.8166, 3.5965, 21.4350]);
    
    % Hand
    I(:,:,5) = diag([2.9383, 2.8319, 0.3549]);
    
    % Flange (忽略)
end

%% 轨迹规划 (五次多项式，每个关节独立)
% 设置每个关节的起始位置和目标位置
q0 = zeros(n_joints, 1); % 起始位置
qf = [pi/2; -pi/3; pi/4; pi/6; -pi/4; pi/3]; % 目标位置

% 初始化轨迹矩阵
q_des = zeros(n_joints, N);
qd_des = zeros(n_joints, N);
qdd_des = zeros(n_joints, N);

% 生成轨迹
for j = 1:n_joints
    % 五次多项式插值
    for k = 1:N
        t_normalized = t(k) / T_total; % 归一化时间 [0,1]
        
        % 位置
        s = 10*t_normalized^3 - 15*t_normalized^4 + 6*t_normalized^5;
        q_des(j,k) = q0(j) + (qf(j) - q0(j)) * s;
        
        % 速度
        ds = (30*t_normalized^2 - 60*t_normalized^3 + 30*t_normalized^4) / T_total;
        qd_des(j,k) = (qf(j) - q0(j)) * ds;
        
        % 加速度
        d2s = (60*t_normalized - 180*t_normalized^2 + 120*t_normalized^3) / T_total^2;
        qdd_des(j,k) = (qf(j) - q0(j)) * d2s;
    end
end

%% ILC
% N = size(u_ff, 2);
assert (N>0, "时间步数无效");

iter_max = 200;        % 最大迭代次数
e_history = zeros(iter_max, N, n_joints);
u_history = zeros(iter_max, N, n_joints);
rmse = zeros(iter_max,1);

% 模型前馈增强
u_ff_model = zeros(n_joints, N);
for k = 1:N
    % 使用标准逆动力学计算前馈力矩
    u_ff_model(:,k) = test_inverse(...
        q_des(:,k), qd_des(:,k), qdd_des(:,k), robot_params);
end

% 模型置信度 (0.8表示对模型有较高信心)
model_confidence = 0.7;
u_ff = model_confidence * u_ff_model;
u_prev = u_ff;

% ILC参数
max_gain = [0.6, 0.5, 0.4, 0.3, 0.2, 0.1];
min_gain = [0.2, 0.15, 0.1, 0.08, 0.06, 0.04];
Q = 0.45; % 低通滤波器系数
Kp = [300, 280, 250, 180, 150, 100]; % 位置增益
Kd = [30, 28, 25, 18, 15, 10]; % 速度增益

%% 完整动力学校验
gravity_tau = test_inverse(q0, zeros(6,1), zeros(6,1), robot_params);
fprintf('静态重力矩校验: [%.1f, %.1f, %.1f, %.1f, %.1f, %.1f] Nm\n', gravity_tau);
q_test = [0; deg2rad(-30); 0; 0; 0; 0];
gravity_tau2 = test_inverse(q_test, zeros(6,1), zeros(6,1), robot_params);
assert(gravity_tau2(2) > 0, '关节2重力矩方向错误!');

% 迭代学习控制主循环
for iter = 1:iter_max
    decay_rate = 0.25;
    min_decay = 0.05;
    Lp = max(min_gain + (max_gain - min_gain).*exp(-decay_rate*iter), min_decay); % P型学习增益
    Li = [0.5, 1, 0.8, 0.3, 0.2, 0.1];
    Ld = 0.15 * Lp; % D型学习增益
    La = [0.07, 0.06, 0.05, 0.04, 0.03, 0.02] .* exp(-0.25*iter);
    q_act = q_des(:,1); % 初始位置
    qd_act = zeros(n_joints,1); % 初始速度
    y = zeros(n_joints, N);
    u_fb = zeros(n_joints, N);
    if iter <= 50
        alpha = 0.7 + 0.2 * tanh(iter/20);  % 初期缓慢增长
    else
        alpha = 0.85;  % 后期固定值，平衡历史/新数据
    end

    % 运行单次迭代
    for k = 1:N-1
        % 反馈控制tau_total
        e = q_des(:,k) - q_act;
        edot = qd_des(:,k) - qd_act;
        u_fb(:,k) = Kp' .* e + Kd' .* edot;

               
        % 摩擦前馈补偿
        friction_comp = zeros(n_joints,1);
        for j = 1:n_joints
            friction_comp(j) = enhancedFrictionModel(...
                qd_des(j,k), qdd_des(j,k), robot_params.fc(j), robot_params.fb(j), ...
                robot_params.fs(j), robot_params.vs(j));
        end
        
        % 总控制力矩
        tau_total = u_ff(:,k) + u_fb(:,k) + friction_comp;

        % 修复前馈力矩历史记录
        if iter == 1
            u_history(iter, :, :) = u_ff'; % 初始化存储
        else
            u_history(iter, :, :) = permute(u_ff, [2,1]); % 保存当前迭代的u_ff
        end
        
        % 正向动力学更新状态
        [q_new, qd_new] = test_forward(...
            q_act, qd_act, tau_total, robot_params, Ts);
        
        % 更新状态
        q_act = q_new;
        qd_act = qd_new;
        y(:,k+1) = q_act;
    end
    
    % 计算跟踪误差
    joint_weights = [0.4, 0.3, 0.2, 0.05, 0.03, 0.02];

    % 加权RMSE
    e_iter = q_des - y;
    joint_rmse = sqrt(mean(e_iter.^2,2));
    noise_level = std(e_iter, 0, 2);
    dynamic_weights = 1./(noise_level + 1e-5);
    dynamic_weights = dynamic_weights / sum(dynamic_weights);
    rmse(iter) = sum(joint_rmse .* dynamic_weights);
    total_weights = sum(joint_weights);
    weighted_sum = dot(joint_rmse,joint_weights);
    rmse(iter) = weighted_sum / total_weights;
    e_history(iter,:,:) = e_iter';
    
    % 收敛判断
    convergence_threshold = 1e-5 + 0.5e-5 * iter;

    min_improve = 0.001; % 最小改进率阈值 (0.1%)
    patience = 10;       % 连续10次未显著改进则停止
    if iter > patience
        improve_ratio = abs(diff(rmse(iter-patience:iter))) ./ rmse(iter-patience:iter-1);
        if all(improve_ratio < min_improve)
            fprintf('早停于迭代%d，平均改进率<%.4f%%\n', iter, min_improve*100);
            break;
        end
    end
    
    % ILC更新律
    if iter < iter_max 
        for j = 1:n_joints
            e_joint = squeeze(e_history(iter,:,j));
            if iscolumn(e_joint)
                e_joint = e_joint';% 转为行向量
            end 

            % 行向量转换 
            if iscolumn(e_joint)
                e_joint = e_joint .';
            end
            
            de = [0, diff(e_joint)]/Ts;

            % 学习律
            delta_u = Lp(j)*e_joint + Ld(j)*de + La(j)*qdd_des(j,:);
            
            % 低通滤波
            delta_u_filt = filter(1-Q, [1, -Q], delta_u);
            
            if iscolumn(delta_u_filt)
                delta_u_filt = delta_u_filt .';
            end
            %% 在滤波后添加硬限幅 (限制增量在±10%范围内)
            max_delta = 0.1 * abs(u_ff(j,:));
            delta_u_filt = min(max(delta_u_filt, -max_delta), max_delta);
            % 替换原有的 u_ff 更新逻辑
            % momentum = 0.3 *(u_ff(j,:) - u_prev(j,:));
            % delta_u_filt = alpha * delta_u_filt;  % 加入遗忘因子
            % u_ff(j,:) = alpha* u_ff(j,:) + delta_u_filt + momentum; % 增量更新
            e_joint = e_joint(:);
            de = de(:);
            delta_u = Lp.*e_joint + Ld.*de;
            N = min(length(delta_u), size(u_ff, 2));  % 取最小公共长度
            delta_u_trunc = delta_u(1:N);             % 截断 delta_u
            u_ff_trunc = u_ff(j, 1:N);                % 截断 u_ff
            delta_u = sign(delta_u_trunc) .* min(abs(delta_u_trunc), 0.1*abs(u_ff_trunc));            
            u_ff(j,:) = alpha*u_ff(j,:) + delta_u;
        end
    end
    
    fprintf('迭代 %3d: 总体RMSE = %.6f rad\n', iter, rmse(iter));
end

%% 可视化结果
% 可视化模块1：轨迹跟踪误差
figure;
for j = 1:3
    subplot(3,1,j);
    hold on;
    plot(t, q_des(j, :), 'r--', 'LineWidth', 2);
    plot(t, y(j, :), 'b-', 'LineWidth', 1.5);
    xlabel('时间 (s)');
    ylabel('位置 (rad)');
    legend('期望', '实际', 'Location', 'best');
    title(sprintf('关节 %d 轨迹跟踪', j));
    grid on;
end

% 可视化模块2：收敛曲线
figure('Name', 'ILC收敛过程');
semilogy(1:iter, rmse(1:iter), 'bo-', 'LineWidth', 1.8, 'MarkerSize', 6, ...
    'MarkerFaceColor', 'b');
grid on;
xlabel('迭代次数');
ylabel('加权RMSE (log scale)');
title('ILC收敛曲线');
xlim([1, iter]);

% 标记收敛阈值
hold on;
plot([1, iter], [convergence_threshold, convergence_threshold], 'r--', 'LineWidth', 1.2);
text(iter*0.6, convergence_threshold*1.2, sprintf('收敛阈值: %.2e', convergence_threshold), ...
    'Color', 'r');

% 添加收敛信息
if iter < iter_max
    text(iter*0.1, rmse(1)*0.8, sprintf('收敛于迭代 %d', iter), 'FontSize', 12, 'Color', 'k');
end

% 可视化模块3：摩擦模型特性
figure('Name', '摩擦模型特性分析', 'Position', [100 100 1000 600]);

% 模拟不同速度下的摩擦力
qd_range = linspace(-2, 2, 500); % ±2 rad/s
tau_friction = zeros(6, length(qd_range));

% 计算各关节摩擦特性
for j = 1:6
    for k = 1:length(qd_range)
        tau_friction(j,k) = enhancedFrictionModel(qd_range(k), 0, ... % 零加速度
            robot_params.fc(j), robot_params.fb(j), ...
            robot_params.fs(j), robot_params.vs(j));
    end
end

% 绘制摩擦曲线
colors = lines(6);
subplot(2,1,1);
for j = 1:6
    plot(qd_range, tau_friction(j,:), 'Color', colors(j,:), 'LineWidth', 1.8);
    hold on;
end
title('关节摩擦力矩 vs 速度');
xlabel('关节速度 (rad/s)');
ylabel('摩擦扭矩 (Nm)');
legend(arrayfun(@(x) sprintf('关节%d', x), 1:6, 'UniformOutput', false));
grid on;

% 绘制Stribeck过渡区细节
subplot(2,1,2);
qd_detail = linspace(-0.2, 0.2, 200); % ±0.2 rad/s
tau_detail = zeros(6, length(qd_detail));

for j = 1:6
    for k = 1:length(qd_detail)
        tau_detail(j,k) = enhancedFrictionModel(qd_detail(k), 0, ...
            robot_params.fc(j), robot_params.fb(j), ...
            robot_params.fs(j), robot_params.vs(j));
    end
    plot(qd_detail, tau_detail(j,:), 'Color', colors(j,:), 'LineWidth', 1.8);
    hold on;
end
title('Stribeck效应区域（低速细节）');
xlabel('关节速度 (rad/s)');
ylabel('摩擦扭矩 (Nm)');
grid on;
xlim([-0.2, 0.2]);

%% 修复的控制信号可视化
if iter >= 2 % 至少有两次迭代数据
    figure('Name', '控制信号分析', 'Position', [100 100 1200 800]);
    
    % 选择最后3次迭代进行分析
    iter_range = max(1, iter-2):iter;
    colors = jet(length(iter_range));
    
    % 分析第3关节（典型关节）
    j = 3;
    
    % 确保时间向量是列向量
    t_col = t(:); % 转换为列向量
    
    % 1. 前馈控制信号变化
    subplot(3,1,1);
    for i = 1:length(iter_range)
        idx = iter_range(i);
        
        % 安全提取控制信号并确保列向量
        control_signal = reshape(squeeze(u_history(idx,j,:)), [], 1);
        
        % 检查维度并截断或填充以匹配
        if length(control_signal) > length(t_col)
            % 信号比时间向量长，截断
            control_signal = control_signal(1:length(t_col));
        elseif length(control_signal) < length(t_col)
            % 信号比时间向量短，填充零
            control_signal = [control_signal; zeros(length(t_col)-length(control_signal), 1)];
        end
        
        % 绘图
        plot(t_col, control_signal, 'Color', colors(i,:), 'LineWidth', 1.5);
        hold on;
    end
    title(sprintf('关节%d - 前馈控制信号演变', j));
    xlabel('时间 (s)');
    ylabel('前馈力矩 (Nm)');
    legend(arrayfun(@(x) sprintf('迭代%d', x), iter_range, 'UniformOutput', false));
    grid on;
    
    % 2. 前馈信号增量
    subplot(3,1,2);
    for i = 2:length(iter_range)
        idx_current = iter_range(i);
        idx_prev = iter_range(i-1);
        
        % 提取信号
        current_sig = reshape(squeeze(u_history(idx_current,j,:)), [], 1);
        prev_sig = reshape(squeeze(u_history(idx_prev,j,:)), [], 1);
        
        % 确保相同长度
        min_len = min(length(current_sig), length(prev_sig));
        current_sig = current_sig(1:min_len);
        prev_sig = prev_sig(1:min_len);
        
        % 计算增量
        delta_u = current_sig - prev_sig;
        
        % 截断时间向量以匹配
        t_delta = t_col(1:min_len);
        
        plot(t_delta, delta_u, 'Color', colors(i,:), 'LineWidth', 1.5);
        hold on;
    end
    title('前馈力矩增量');
    xlabel('时间 (s)');
    ylabel('力矩增量 (Nm)');
    grid on;
    
    % 3. 摩擦补偿信号
    subplot(3,1,3);
    friction_com = zeros(length(t_col),1); % 列向量
    for k = 1:length(t_col)
        % 确保索引不越界
        k_idx = min(k, size(qd_des, 2));
        friction_com(k) = enhancedFrictionModel(...
            qd_des(j,k_idx), qdd_des(j,k_idx), ...
            robot_params.fc(j), robot_params.fb(j), ...
            robot_params.fs(j), robot_params.vs(j));
    end
    plot(t_col, friction_com, 'm-', 'LineWidth', 1.8);
    title('摩擦补偿力矩');
    xlabel('时间 (s)');
    ylabel('补偿力矩 (Nm)');
    grid on;
    ylim([min(friction_com)*1.1, max(friction_com)*1.1]);
end

%% 逆向动力学函数
function tau = test_inverse(q, qd, qdd, params)
    % 提取参数
    DH_params = params.DH;
    n = length(q);
    a = DH_params(:,1);
    alpha = DH_params(:,2);
    d = DH_params(:,3);
    theta = DH_params(:, 4);
    m = params.m;
    r = params.cm_pos;
    I = params.I;
    g = params.g';
    
    % 初始化变量
    w = zeros(3,1);       % 基座角速度
    dw = zeros(3,1);      % 基座角加速度
    vd = g;               % 基座线加速度(含重力)
    
    % 前向递推 (计算速度和加速度)
    w_save = zeros(3,n+1);
    dw_save = zeros(3,n+1);
    vd_save = zeros(3,n+1);
    ac = zeros(3,n);       % 质心加速度
    
    w_save(:,1) = w;
    dw_save(:,1) = dw;
    vd_save(:,1) = vd;
    
    for i = 1:n
        % DH变换参数
        ct = cos(theta(i));
        st = sin(theta(i));
        ca = cos(alpha(i));
        sa = sin(alpha(i));
        
        % 旋转矩阵 (i-1到i)
        R = [ct, -st*ca,  st*sa;
             st,  ct*ca, -ct*sa;
              0,     sa,     ca];
        
        % P向量 (在i-1坐标系中)
        P = [a(i); d(i)*sa; d(i)*ca];
        
        % 计算角速度和角加速度
        w_prev = w_save(:,i);
        w_rel = [0; 0; qd(i)];
        w_next = R*w_prev + w_rel;
        
        dw_prev = dw_save(:,i);
        dw_rel = [0; 0; qdd(i)];
        dw_next = R*dw_prev + cross(R*w_prev, w_rel) + dw_rel;
        
        % 计算线加速度
        vd_prev = vd_save(:,i);
        vd_i = R*(vd_prev + cross(dw_prev, P) + ...
               cross(w_prev, cross(w_prev, P)));
        
        % 质心加速度
        ac(:,i) = vd_i + cross(dw_next, r(i,:)') + ...
                 cross(w_next, cross(w_next, r(i,:)'));
        
        % 保存结果
        w_save(:,i+1) = w_next;
        dw_save(:,i+1) = dw_next;
        vd_save(:,i+1) = vd_i;
    end
    
    % 后向递推 (计算力和力矩)
    f = zeros(3,n+1);     % 关节力
    n_torque = zeros(3,n+1); % 关节力矩
    
    % 末端执行器无外力
    f(:,n+1) = [0;0;0];
    n_torque(:,n+1) = [0;0;0];
    
    tau = zeros(n,1);
    
    for i = n:-1:1
        % DH参数
        ct = cos(theta(i));
        st = sin(theta(i));
        ca = cos(alpha(i));
        sa = sin(alpha(i));
        
        R = [ct, -st*ca,  st*sa;
             st,  ct*ca, -ct*sa;
              0,     sa,     ca];
        
        P = [a(i); d(i)*sa; d(i)*ca]; % 注意转置
        
        % 从下一个坐标系转换到当前坐标系
        f_next = f(:,i+1);
        n_next = n_torque(:,i+1);
        
        % 牛顿方程 (质心)
        F = m(i) * ac(:,i);
        f_i = R*f_next + F;
        
        % 欧拉方程 (质心)
        N = I(:,:,i)*dw_save(:,i+1) + ...
            cross(w_save(:,i+1), I(:,:,i)*w_save(:,i+1));
        
        % 力矩平衡
        n_i = R*n_next + cross(P, R*f_next) + ...
              cross(r(i,:)', F) + N;
        
        % 关节力矩
        tau(i) = n_i(3); % 绕z轴的力矩
        
        % 保存结果
        f(:,i) = f_i;
        n_torque(:,i) = n_i;
    end
    
    % 添加摩擦力模型
    for i = 1:n
        tau_friction = coulombViscousFriction(qd(i), ...
                      params.fc(i), params.fb(i));
        tau(i) = tau(i) + tau_friction;
    end
    
    % 考虑减速比和电机惯量
    for i = 1:n
        gr = params.gear_ratio(i);
        Jm = params.motor_inertia(i);
        % % 考虑电机转子加速度的真实效应
        % rotor_acc = qdd(i) * abs(gr);
        % if gr < 0
        %     %tau(i) = tau(i)/abs(gr) - Jm * rotor_acc;
        %     tau(i) = tau(i)/abs(gr) - Jm * abs(gr) * qdd(i);
        % else
        %     %tau(i) = tau(i)/abs(gr) + Jm * rotor_acc;
        %     tau(i) = tau(i)/abs(gr) + Jm * abs(gr) * qdd(i);
        % end
        motor_effect = Jm* gr* qdd(i);
        tau(i) = tau(i)/abs(gr) + motor_effect;
    end
    
    % 辅助函数: 连续摩擦模型
    function tau_f = coulombViscousFriction(qd, fc, fb)
        % 连续可导的摩擦模型
        beta = 100; % 光滑因子
        tau_f = (2*fc/(1+exp(-beta*qd)) - fc) + fb*qd;
    end
end

%% 正向动力学函数
function [q_new, qd_new] = test_forward(q, qd, tau, params, Ts)
    n = length(q);
    
    % 使用逆动力学计算广义质量矩阵M
    M = computeMassMatrix(q, params);
    
    % 计算速度项 (C + G)
    h = computeVelocityTerms(q, qd, params);
    
    % 计算加速度 (qdd = M^{-1}(tau - h))
    %qdd = lsqminnorm(M, tau-h); % 最小范数最小二乘法
    
    cond_M = cond(M);
    lambda = 1e-6* min(1, cond_M/1e10);
    M_reg = M + lambda * eye(size(M));  % 正则化法
    % qdd = M_reg \ (tau-h);
    
    if cond_M > 1e15
        qdd = pinv(M_reg) * (tau-h);
    else
        qdd = M_reg \ (tau-h);
    end

    % tol = 1e-8;
    % qdd = pinv(M,tol)*(tau-h); % 截断小奇异值
    
    % 状态更新 (半隐式欧拉)
    qd_new = qd + qdd*Ts;
    q_new = q + qd*Ts + 0.5*qdd*Ts^2;
    
    % 关节限位处理
    for i = 1:n
        q_new(i) = max(min(q_new(i), params.joint_limits(i,2)), ...
                       params.joint_limits(i,1));
        qd_new(i) = sign(qd_new(i))*min(abs(qd_new(i)), ...
                   deg2rad(params.max_velocity(i)));
    end
    
    % 辅助函数: 计算质量矩阵
    function M = computeMassMatrix(q, params)
        n = length(q);
        M = zeros(n);
        T = forwardKinematics(q,params);

        p_cm = zeros(3,n);
        for i = 1:n
            R = T(1:3,1:3,i);
            p_cm(:,i) = T(1:3,4,i) + R*params.cm_pos(i,:)';
        end

        Jv = zeros(3,n,n); %[3*n_joints*n_links]
        Jw = zeros(3,n,n);
        for k =1:n
            for j = 1:k
                z_prev = [0;0;1];
                if j>1
                    R_prev = T(1:3,1:3,j-1);
                    z_prev = R_prev*[0;0;1];
                end
                
                if params.joint_type(j) =='R'
                    p_j = T(1:3,4,j);
                    Jv(:,j,k) = cross(z_prev,p_cm(:,k)-p_j);
                    Jw(:,j,k) = z_prev;
                else
                    Jv(:,j,k) = z_prev;
                end
            end
        end
        for i = 1:n
            for j = 1:n
                for k = max(i,j):n
                    trans_term = params.m(k) *(Jv(:,i,k)'* Jv(:,j,k));
                    rot_term = Jw(:,i,k)' * params.I(:,:,k) * Jw(:,j,k);
                    M(i,j) = M(i,j) + trans_term + rot_term;
                end
            end
        end
    end
    
    % 正运动学函数
    function T = forwardKinematics(q, params)
    DH = params.DH;
    n = size(DH,1);
    T = zeros(4,4,n); % 存储所有变换矩阵
    
    T_prev = eye(4);
    for i = 1:n
        a = DH(i,1); alpha = DH(i,2); d = DH(i,3); theta = DH(i,4) + q(i);
        % 标准DH变换矩阵
        Ti = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
              sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
              0,          sin(alpha),             cos(alpha),            d;
              0,          0,                      0,                     1];
        T(:,:,i) = T_prev * Ti;
        T_prev = T(:,:,i);
    end
end

    
    % 辅助函数: 计算速度项
    function h = computeVelocityTerms(q, qd, params)
        zero_acc = zeros(length(q),1);
        h = test_inverse(q, qd, zero_acc, params);
    end
end

%% 增强摩擦模型函数
function tau_f = enhancedFrictionModel(qd, qdd, fc, fb, fs, vs)
    % 统一Stribeck模型（避免非连续跳变）
    beta = 50;
    stribeck = fc + (fs - fc) * exp(-(abs(qd)/vs)^0.7);
    % 加速度补偿项
    viscous = fb * qd;
    w = 1./ (1 + exp(-beta*(abs(qd) - 05*vs)));
    tau_f = sign(qd) .* ((1-w).*stribeck + w.*(fc + viscous));
    
end
%% 摩擦前馈计算，已被上位替代
function tau_f = computeFrictionFeedforward(qd, fc, fb)
    % 连续可导的摩擦前馈模型
    if abs(qd) > 0.01
        tau_f = fc*sign(qd) + fb*qd;
    else
        % 低速区平滑过渡
        tau_f = 2*fc*qd/0.01 + fb*qd;
    end
end