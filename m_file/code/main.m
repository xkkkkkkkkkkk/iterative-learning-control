clear; clc; close all;

%% 初始化参数
% urdf 模型
robot  = importrobot('kr10_r1100_2_urdf.urdf'); 
robot.DataFormat = 'column';
robot.Gravity = [0, 0, -9.81];

% DH参数
DH_Parameter = [   0     pi/2     10        0; 
                  60        0      0        0; 
                   0     pi/2      0     pi/2; 
                   0    -pi/2     50        0;
                   0     pi/2      0        0; 
                   0        0     20        0   ];  

% 轨迹参数
totalTime = 5; % 总时间(秒)
sampleRate = 100; % 采样率(Hz)
t = linspace(0, totalTime, totalTime*sampleRate);
numIteration = 25; % 最大迭代次数

% 关节轨迹数组
numPoints = length(t);
numJoints = 6; % 6自由度机械臂
jointAngles = zeros(numPoints, numJoints);
jointVelocity = zeros(numPoints, numJoints);

% ILC参数
% Kp = [0.1, 0.1, 0.1, 0.1, 0.1, 0.01];
Kp = 0.01;
% Kd = [0.01, 0.01, 0.01, 0.01, 0.01, 0.001];
Kd = 0.001;
err = zeros(numPoints, numJoints, numIteration);
U_contorl = zeros(numPoints, numJoints);
desired_threshold = 0.1;

% 定义起始和目标位姿（笛卡尔）
startPose = [50, 50, 50, deg2rad(30), deg2rad(60), deg2rad(45)]; % [x,y,z,roll,pitch,yaw]
endPose = [10, 0, 20, deg2rad(30), deg2rad(60), deg2rad(45)];

%% 生成末端执行器轨迹并转换为关节角度
[desired_pos, desired_euler, desired_vel, desired_acc, desired_omega, desired_alpha] = ...
    GenerateDesiredTrajectory(t, 'pointToPoint', startPose, endPose);

% 为每个轨迹点计算逆运动学
fprintf('开始逆运动学计算...\n');
for i = 1:numPoints
    % 提取当前点的位姿
    currentPos = desired_pos(i, :)';
    currentEuler = desired_euler(i, :)';
    
    % 逆运动学求解存储
    jointAngles(i, :) = InverseKinematics(currentPos, currentEuler, numJoints, DH_Parameter);
    
    % 显示进度
    if mod(i, 50) == 0
        fprintf('已完成 %.1f%%\n', (i/numPoints)*100);
    end
end

%% ILC迭代循环
initConfig = homeConfiguration(robot);
q0 = jointAngles(1,:)';
dq0 = zeros(6, 1);
init_state = zeros(2*6, 1);
init_state(1:2:end) = q0;
init_state(2:2:end) = dq0;
options = odeset('RelTol', 1e-6, 'AbsTol', 1e-10);
fprintf('开始ILC迭代循环...\n');
for k = 1:numIteration
    U_input = U_contorl;
    if k >1
        Delta_U = ILC_update(err(:,:,k-1), t, Kp, Kd );
        U_input = U_input + Delta_U;
    end

    % 调用ODE45之前，定义力矩插值函数
    tau_func = @(time) [interp1(t, U_input(:,1), time)'; ... % 插值得到tau1
        interp1(t, U_input(:,2), time)'; ... % 插值得到tau2
        interp1(t, U_input(:,3), time)'; ... % 插值得到tau3
        interp1(t, U_input(:,4), time)'; ... % 插值得到tau4
        interp1(t, U_input(:,5), time)'; ... % 插值得到tau5
        interp1(t, U_input(:,6), time)'];    % 插值得到tau6

    % Q_actual = simulate_robot_model(U_input, t);

    [t_sim, state_history] = ode15s(@(t,state) robot_dynamics_ode(t, state, tau_func(t), robot), t, init_state, options);
    Q_actual = state_history(:, 1:2:12);

    e_k = jointAngles - Q_actual;
    err(:, :, k) = e_k;
    U_contorl = U_input;

    rms_err(k) = rms(e_k(:));
    if rms_err(k) < desired_threshold
        fprintf('收敛于 %d 次 \n', k);
        break;
    end 
    fprintf('已完成第%.1f次迭代\n', k);
end

% 单独测试动力学模型
% test_tau = zeros(length(t), 6); % 生成一个全零的测试力矩输入
% test_tau_func = @(time) [interp1(t, test_tau(:,1), time)';
%                          interp1(t, test_tau(:,2), time)';
%                          interp1(t, test_tau(:,3), time)';
%                          interp1(t, test_tau(:,4), time)';
%                          interp1(t, test_tau(:,5), time)';
%                          interp1(t, test_tau(:,6), time)';];
% fprintf('开始运行单独动力学测试...\n');
% [test_t_sim, test_state_history] = ode45(@(t,state) robot_dynamics_ode(t, state, test_tau_func(t), robot), t, init_state, options);
% fprintf('单独动力学测试完成。\n');
% plot(test_t_sim, test_state_history(:,1:6));
% title('单独动力学测试：零输入响应');

%% 可视化结果
% 1. 绘制各关节误差随时间变化曲线
figure;
hold on;
colors = lines(numJoints); 
for joint = 1:numJoints
    plot(t, err(:, joint, end), 'Color', colors(joint, :), 'LineWidth', 1.5, ...
        'DisplayName', ['关节 ', num2str(joint)]);
end
hold off;
xlabel('时间 (s)');
ylabel('关节角度误差 (rad)');
title('各关节跟踪误差随时间变化');
legend('show');
grid on;

% 2. 绘制最后一次迭代的误差RMS值
figure;
rms_per_joint = zeros(numJoints, 1);
for joint = 1:numJoints
    rms_per_joint(joint) = rms(err(:, joint, end));
end
bar(rms_per_joint);
xlabel('关节编号');
ylabel('RMS误差 (rad)');
title('各关节误差RMS值');
grid on;

% 3. 绘制每次迭代的总体RMS误差变化
figure;
plot(1:numIteration, rms_err, 'bo-', 'LineWidth', 1.5);
xlabel('迭代次数');
ylabel('总体RMS误差 (rad)');
title('ILC收敛过程');
grid on;

% 4. 绘制期望轨迹与实际轨迹对比（三维空间）
figure;
hold on;

% 计算期望末端轨迹
desired_positions = zeros(numPoints, 3);
actual_positions = zeros(numPoints, 3);

% 通过正运动学计算位置
for i = 1:numPoints
    % 期望位置
    T_desired = ForwardKinematics(numJoints, jointAngles(i, :), DH_Parameter);
    desired_positions(i, :) = T_desired.P';
    
    % 实际位置（最后一次迭代）
    T_actual = ForwardKinematics(numJoints, Q_actual(i, :), DH_Parameter);
    actual_positions(i, :) = T_actual.P';
end

plot3(desired_positions(:,1), desired_positions(:,2), desired_positions(:,3), ...
    'b-', 'LineWidth', 2, 'DisplayName', '期望轨迹');
plot3(actual_positions(:,1), actual_positions(:,2), actual_positions(:,3), ...
    'r', 'LineWidth', 2, 'DisplayName', '实际轨迹');


% 标记起点和终点
plot3(desired_positions(1,1), desired_positions(1,2), desired_positions(1,3), ...
    'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g', 'DisplayName', '起点');
plot3(desired_positions(end,1), desired_positions(end,2), desired_positions(end,3), ...
    'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r', 'DisplayName', '终点');

hold off;
xlabel('X (mm)');
ylabel('Y (mm)');
zlabel('Z (mm)');
title('末端执行器轨迹对比');
legend('show');
grid on;
axis equal;
