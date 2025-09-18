function [pos, euler, vel, acc, omega, alpha] = GenerateDesiredTrajectory(t, trajType, varargin)
% GENERATEDESIREDTRAJECTORY 生成机械臂末端的期望轨迹
%   输入:
%       t        - 时间向量 (例如: linspace(0, 10, 100))
%       trajType - 轨迹类型: 'pointToPoint', 'bezier', 'circular', 'custom'
%       varargin - 可变参数，依赖于轨迹类型
%   输出:
%       pos      - 位置 [x, y, z] (Nx3)
%       euler    - 欧拉角 [roll, pitch, yaw] (弧度, Nx3)
%       vel      - 线速度 [vx, vy, vz] (Nx3)
%       acc      - 线加速度 [ax, ay, az] (Nx3)
%       omega    - 角速度 [ωx, ωy, ωz] (Nx3)
%       alpha    - 角加速度 [αx, αy, αz] (Nx3)
%
%   示例调用:
%      [p, e, v, a, w, alpha] = GenerateDesiredTrajectory(t, 'pointToPoint', startPose, endPose);
%      [p, e, v, a, w, alpha] = GenerateDesiredTrajectory(t, 'bezier', controlPoints);
%      [p, e, v, a, w, alpha] = GenerateDesiredTrajectory(t, 'circular', center, radius, normalVec);
%      [p, e, v, a, w, alpha] = GenerateDesiredTrajectory(t, 'custom', @myTrajFunc);

    % 基本参数设置
    N = length(t);
    dt = t(2) - t(1); % 假设时间均匀间隔
    
    % 初始化输出数组
    pos = zeros(N, 3);
    euler = zeros(N, 3);
    vel = zeros(N, 3);
    acc = zeros(N, 3);
    omega = zeros(N, 3);
    alpha = zeros(N, 3);
    
    % 根据轨迹类型生成路径
    switch trajType
        case 'pointToPoint'
            % 参数: startPose, endPose (各为6元素向量: [x,y,z,roll,pitch,yaw])
            startPose = varargin{1};
            endPose = varargin{2};
            
            % 使用五次多项式插值，保证加速度连续[3](@ref)
            for i = 1:6
                [q, qd, qdd] = quinticPolynomial(t, startPose(i), endPose(i));
                if i <= 3
                    pos(:, i) = q;
                    vel(:, i) = qd;
                    acc(:, i) = qdd;
                else
                    euler(:, i-3) = q;
                    omega(:, i-3) = qd;
                    alpha(:, i-3) = qdd;
                end
            end
            
        case 'bezier'
            % 参数: controlPoints (Mx3矩阵，M>=4对于3次贝塞尔曲线)[1](@ref)
            controlPoints = varargin{1};
            M = size(controlPoints, 1);
            
            % 生成贝塞尔曲线路径[1](@ref)
            s = linspace(0, 1, N); % 归一化参数
            for i = 1:N
                pos(i, :) = bezierCurve(controlPoints, s(i));
            end
            
            % 数值微分计算速度和加速度
            vel(2:end, :) = diff(pos) / dt;
            vel(1, :) = vel(2, :);
            acc(2:end, :) = diff(vel) / dt;
            acc(1, :) = acc(2, :);
            
            % 假设姿态固定或单独指定
            if nargin > 3
                attitudeFunc = varargin{2};
                for i = 1:N
                    euler(i, :) = attitudeFunc(s(i));
                end
            else
                euler(:, :) = repmat([0, 0, 0], N, 1); % 默认零姿态
            end
            
            % 数值微分计算角速度和角加速度
            omega(2:end, :) = diff(euler) / dt;
            omega(1, :) = omega(2, :);
            alpha(2:end, :) = diff(omega) / dt;
            alpha(1, :) = alpha(2, :);
            
        case 'circular'
            % 参数: center, radius, normalVec[3](@ref)
            center = varargin{1};
            radius = varargin{2};
            normalVec = varargin{3};
            
            % 生成圆形路径
            theta = linspace(0, 2*pi, N);
            pos(:, 1) = center(1) + radius * cos(theta);
            pos(:, 2) = center(2) + radius * sin(theta);
            pos(:, 3) = center(3);
            
            % 计算速度和加速度
            vel(:, 1) = -radius * sin(theta) * (2*pi/(t(end)-t(1)));
            vel(:, 2) = radius * cos(theta) * (2*pi/(t(end)-t(1)));
            acc(:, 1) = -radius * cos(theta) * (2*pi/(t(end)-t(1)))^2;
            acc(:, 2) = -radius * sin(theta) * (2*pi/(t(end)-t(1)))^2;
            
            % 使姿态始终朝向圆心或切线方向
            for i = 1:N
                % 这是一个简化示例，实际应根据需要定义姿态策略
                euler(i, :) = [0, 0, atan2(vel(i, 2), vel(i, 1))];
            end
            
            % 数值微分计算角速度和角加速度
            omega(2:end, :) = diff(euler) / dt;
            omega(1, :) = omega(2, :);
            alpha(2:end, :) = diff(omega) / dt;
            alpha(1, :) = alpha(2, :);
            
        case 'custom'
            % 参数: 自定义函数句柄，函数应接受时间返回位姿
            customFunc = varargin{1};
            
            % 计算位置和姿态
            for i = 1:N
                [pos(i, :), euler(i, :)] = customFunc(t(i));
            end
            
            % 数值微分计算所有导数
            vel(2:end, :) = diff(pos) / dt;
            vel(1, :) = vel(2, :);
            acc(2:end, :) = diff(vel) / dt;
            acc(1, :) = acc(2, :);
            
            omega(2:end, :) = diff(euler) / dt;
            omega(1, :) = omega(2, :);
            alpha(2:end, :) = diff(omega) / dt;
            alpha(1, :) = alpha(2, :);
            
        otherwise
            error('不支持的轨迹类型');
    end
end

% 五次多项式插值函数
function [q, qd, qdd] = quinticPolynomial(t, q0, qf)
    tf = t(end);
    a0 = q0;
    a1 = 0;
    a2 = 0;
    a3 = 10*(qf - q0)/(tf^3);
    a4 = -15*(qf - q0)/(tf^4);
    a5 = 6*(qf - q0)/(tf^5);
    
    q = a0 + a1*t + a2*t.^2 + a3*t.^3 + a4*t.^4 + a5*t.^5;
    qd = a1 + 2*a2*t + 3*a3*t.^2 + 4*a4*t.^3 + 5*a5*t.^4;
    qdd = 2*a2 + 6*a3*t + 12*a4*t.^2 + 20*a5*t.^3;
end

% 贝塞尔曲线计算函数[1](@ref)
function point = bezierCurve(controlPoints, s)
    n = size(controlPoints, 1) - 1;
    point = zeros(1, 3);
    
    for i = 0:n
        term = factorial(n) / (factorial(i) * factorial(n-i)) * ...
               (s^i) * (1-s)^(n-i) * controlPoints(i+1, :);
        point = point + term;
    end
end

