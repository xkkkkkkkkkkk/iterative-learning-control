function dstate = robot_dynamics_ode(t, state, tau, robot)
%   输入:
%       t: 当前时间
%       state: 当前状态向量 [q1; dq1; q2; dq2; ... ; qN; dqN] (2Nx1 column vector)
%       tau: 当前时间步的关节力矩/力输入 
%       robot: rigidBodyTree对象
%   输出:
%       dstate: 状态导数 [dq1; ddq1; dq2; ddq2; ... ; dqN; ddqN] (2Nx1)

    % 1. 动态获取机器人关节数量
    nJoints = 6; 

    % 2. 从状态向量中提取关节位置和速度
    q = state(1:2:end);   % 提取所有关节位置 (Nx1)
    dq = state(2:2:end);  % 提取所有关节速度 (Nx1)


    % 3. 计算机器人动力学
    % 计算质量矩阵M(q) (N x N)
    M = massMatrix(robot, q); 
    
    % 计算科里奥利力、离心力、重力向量 (C(q,dq) + G(q)) (Nx1)
    
    % gravity_only = inverseDynamics(robot, q, zeros(size(dq)), zeros(size(dq)));
    % 调试输出：比较控制力矩与重力补偿需求
    % if t > 0.1 && t < 0.2  % 只在前0.2秒输出，避免信息过多
    %     fprintf('t=%.2f: 控制力矩=[%.3f,%.3f,%.3f], 重力需求=[%.3f,%.3f,%.3f], 差值=[%.3f,%.3f,%.3f]\n', ...
    %         t, tau(1), tau(2), tau(3), gravity_only(1), gravity_only(2), gravity_only(3), ...
    %         tau(1)-gravity_only(1), tau(2)-gravity_only(2), tau(3)-gravity_only(3));
    % end

    tau_coriolis_gravity = inverseDynamics(robot, q, dq, zeros(size(dq))); 
    
    % 4. 求解关节加速度
    dampingCoefficients = [1.5, 1.0, 0.7, 0.3, 0.2, 0.1];
    tau_damping = -dampingCoefficients(:) .* dq;
    ddq = M \ (tau - tau_coriolis_gravity + tau_damping);
    % ddq = M \ (tau - tau_coriolis_gravity);
    
    % if t > 4.5 && t < 5  
    %     disp('质量矩阵 M:');
    %     disp(M);
    %     disp('科里奥利/重力项 tau_coriolis_gravity:');
    %     disp(tau_coriolis_gravity);
    %     disp('计算出的加速度 ddq:');
    %     disp(ddq);
    % end

    % 5. 构建状态导数向量 [速度; 加速度]
    dstate = zeros(2*nJoints, 1);
    for i = 1:nJoints
        dstate(2*i-1) = dq(i);     % 位置导数 = 速度
        dstate(2*i) = ddq(i);      % 速度导数 = 加速度
    end

    % if t > 4.5 && t < 5
    %     fprintf('t=%.3f, q3=%.3f, dq3=%.3f, tau3=%.3f, ddq3=%.3f\n', t, q(3), dq(3), tau(3), ddq(3));
    % end

end