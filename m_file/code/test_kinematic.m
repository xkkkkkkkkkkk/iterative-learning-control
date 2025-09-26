% 初始和目标末端位姿
X_start = 50;    Y_start = 50;    Z_start = 90;
Roll_start = 53; Pitch_start = 37; Yaw_start = 45;

X_end = 20;    Y_end = 30;    Z_end = 35;
Roll_end = 30;  Pitch_end = 60;  Yaw_end = -20;

% 手臂的自由度
DOF = 6;     
% 六轴的 DH 參数  [ a     α        d       θ ]
DH_Parameter = [   0     pi/2     10        0; 
                  60        0      0        0; 
                   0     pi/2      0     pi/2; 
                   0    -pi/2     50        0;
                   0     pi/2      0        0; 
                   0        0     20        0   ]; 

% 1. 计算起始点和目标点的关节角度
JointAngle_start = InverseKinematics([X_start; Y_start; Z_start], [Roll_start, Pitch_start, Yaw_start], DOF, DH_Parameter);
JointAngle_end = InverseKinematics([X_end; Y_end; Z_end], [Roll_end, Pitch_end, Yaw_end], DOF, DH_Parameter);

% 2. 设置轨迹参数
totalTime = 5; % 总运动时间，秒
sampleRate = 100; % 采样频率，Hz
numSamples = totalTime * sampleRate; % 总采样点数
timeVector = linspace(0, totalTime, numSamples)'; % 时间向量

% 3. 为每个关节生成轨迹（这里使用简单的线性插值，实践中常用五次多项式）
trajectory = zeros(numSamples, DOF);
for i = 1:DOF
    trajectory(:, i) = linspace(JointAngle_start(i), JointAngle_end(i), numSamples);
end

% 4. 绘制机械臂运动轨迹
clf;
hold on;
for i = 1:numSamples
    % 获取当前时刻的关节角度
    currentJointAngle = trajectory(i, :);
    
    % 计算正运动学用于绘制
    Info = ForwardKinematics(DOF, currentJointAngle, DH_Parameter);
    
    
    % 可选：绘制末端轨迹
    plot3(Info.P(1), Info.P(2), Info.P(3), 'r.', 'MarkerSize', 2);
    
    title(sprintf('Trajectory Time: %.2f s', timeVector(i)));
    drawnow; % 刷新图形
    % pause(0.01); % 可选：减慢动画速度
end
hold off;