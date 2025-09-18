clear; clc; close all;

%% 初始化参数
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
numIteration = 50; % 最大迭代次数

% 关节轨迹数组
numPoints = length(t);
numJoints = 6; % 6自由度机械臂
jointAngles = zeros(numPoints, numJoints);
jointVelocity = zeros(numPoints, numJoints);

% ILC参数
Kp = 0.1;
Kd = 0.01;
err = zeros(numPoints, numJoints, numIteration);
U_contorl = zeros(numPoints, numJoints);
desired_threshold = 1e-3;

% 定义起始和目标位姿（笛卡尔）
startPose = [50, 50, 90, deg2rad(30), deg2rad(60), deg2rad(45)]; % [x,y,z,roll,pitch,yaw]
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
    % if mod(i, 50) == 0
    %     fprintf('已完成 %.1f%%\n', (i/numPoints)*100);
    % end
end

%% ILC迭代循环
for k = 1:numIteration
    U_input = U_contorl;
    if k >1
        Delta_U = ILC_update(err(:,:,k-1), t, Kp, Kd );
        U_input = U_input + Delta_U;
    end
    Q_actual = simulate_robot_model(U_input, t);
    e_k = jointAngles - Q_actual;
    err(:, :, k) = e_k;
    U_contorl = U_input;

    rms_err(k) = rms(e_k(:));
    if rms_err(k) < desired_threshold
        fprintf('收敛于 %d 次 \n', k);
        break;
    end
    % plot(t, jointAngles, Q_actual, k);
end

% 4. 可视化结果
figure('Name', '关节轨迹', 'Color', 'white', 'Position', [100, 100, 1200, 800]);
subplot(2, 1, 1);
plot(t, rad2deg(jointAngles), 'LineWidth', 1.5);
title('关节角度轨迹');
xlabel('时间 (s)');
ylabel('角度 (°)');
legend('关节1', '关节2', '关节3', '关节4', '关节5', '关节6');
grid on;
fprintf('时间向量 t 的长度: %d\n', length(t));


for i = 1:6
    jointVelocity(:,i) = gradient(jointAngles(:, i), t);
end

subplot(2, 1, 2);
plot(t, jointVelocity, 'LineWidth', 1.5);
title('关节角速度');
xlabel('时间 (s)');
ylabel('角速度 (rad/s)');
grid on;

% 验证轨迹：通过正运动学验证几个关键点
fprintf('验证轨迹精度...\n');
checkPoints = [1, round(numPoints/2), numPoints];
for i = 1:length(checkPoints)
    idx = checkPoints(i);
    jointAngles = jointAngles(idx, :);
    
    % 计算正运动学
    Info = ForwardKinematics(numJoints, jointAngles, DH_Parameter);
    
    % 计算误差
    posError = norm(desired_pos(idx, :) - Info.P');
    orientError = norm(desired_euler(idx, :) - [Info.Roll, Info.Pitch, Info.Yaw]);
    
    fprintf('点 %d: 位置误差=%.4f mm, 姿态误差=%.4f rad\n', idx, posError, orientError);
end