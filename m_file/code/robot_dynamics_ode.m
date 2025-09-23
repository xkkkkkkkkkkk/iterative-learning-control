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
    tau_coriolis_gravity = inverseDynamics(robot, q, dq, zeros(size(dq))); 
    
    % 4. 求解关节加速度
    dampingCoefficients = [1.5, 1.0, 0.7, 0.3, 0.2, 0.1];
    tau_damping = -dampingCoefficients(:) .* dq;
    ddq = M \ (tau - tau_coriolis_gravity + tau_damping);
    
    % if t > 0.3 && t < 0.35  
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

    % if t > 1.0 && t < 1.5
    %     fprintf('t=%.3f, q6=%.3f, dq6=%.3f, tau6=%.3f, ddq6=%.3f\n', t, q(6), dq(6), tau(6), ddq(6));
    % end

end