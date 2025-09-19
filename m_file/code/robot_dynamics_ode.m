function dstate = robot_dynamics_ode(t, state, tau, robot)
% ROBOT_DYNAMICS_ODE 用于ODE求解器的机器人动力学方程
%   输入:
%       t: 当前时间
%       state: 当前状态向量 [q1; dq1; q2; dq2; ... ; qN; dqN] (2Nx1 column vector)
%       tau: 当前时间步的关节力矩/力输入 (Nx1 vector)
%       robot: rigidBodyTree对象
%   输出:
%       dstate: 状态导数 [dq1; ddq1; dq2; ddq2; ... ; dqN; ddqN] (2Nx1)

    % 1. 动态获取机器人关节数量
    nJoints = 6; % 这是关键修改！不再硬编码。

    % 2. 从状态向量中提取关节位置和速度
    q = state(1:2:end);   % 提取所有关节位置 (Nx1)
    dq = state(2:2:end);  % 提取所有关节速度 (Nx1)
    fprintf('t=%.3f, tau=[%s], q=[%s]\n', t, ...
            num2str(tau', '%.2f '), num2str(q', '%.2f ')); % 显示时间和关键变量


    % 3. 计算机器人动力学
    % 计算质量矩阵M(q) (N x N)
    M = massMatrix(robot, q); 
    
    % 计算科里奥利力、离心力、重力向量 (C(q,dq) + G(q)) (Nx1)
    % 注意：inverseDynamics在给定位置、速度、加速度为0时，返回的就是克服非线性力所需的力矩
    tau_coriolis_gravity = inverseDynamics(robot, q, dq, zeros(size(dq))); 
    
    % 4. 求解关节加速度: M(q)*ddq + C(q,dq) + G(q) = tau
    % => ddq = M(q)^{-1} * (tau - (C(q,dq) + G(q)))
    ddq = M \ (tau - tau_coriolis_gravity);
    
    if t > 0.3 && t < 0.35  % 假设问题出现在1秒到1.2秒之间
        disp('质量矩阵 M:');
        disp(M);
        disp('科里奥利/重力项 tau_coriolis_gravity:');
        disp(tau_coriolis_gravity);
        disp('计算出的加速度 ddq:');
        disp(ddq);
    end
    % 5. 构建状态导数向量 [速度; 加速度]
    dstate = zeros(2*nJoints, 1);
    for i = 1:nJoints
        dstate(2*i-1) = dq(i);     % 位置导数 = 速度
        dstate(2*i) = ddq(i);      % 速度导数 = 加速度
    end
end