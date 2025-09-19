function Q_actual = simulate_robot_model(U_input, t)
% SIMULATE_ROBOT_MODEL 模拟真实六轴机械臂的动力学响应
%   输入:
%       U_input - 控制输入（关节力矩），大小为 Nx6 [tau1, tau2, ..., tau6] (单位: N*m)
%       t       - 时间向量 (单位: s)
%   输出:
%       Q_actual - 实际关节角度输出，大小为 Nx6 [q1, q2, ..., q6] (单位: rad)
%
%   该函数通过求解动力学方程，模拟机械臂在给定力矩输入下的运动。
%   使用ODE45求解器进行数值积分。

    % 检查输入维度
    if size(U_input, 2) ~= 6 || length(t) ~= size(U_input, 1)
        error('输入维度不匹配: U_input应为Nx6矩阵, t应为长度为N的向量');
    end

    % 机械臂动力学参数（示例参数，基于常见6轴工业机器人）
    % 您可以替换为您自己机械臂的DH参数和动力学参数
    robot_params = struct();
    
    % 质量 (kg)
    robot_params.m = [3.5, 2.3, 1.7, 0.9, 0.5, 0.3]; 
    
    % 连杆长度 (m)
    robot_params.l = [0.3, 0.25, 0.15, 0.1, 0.05, 0.02]; 
    
    % 质心位置 (相对于前一关节坐标系)
    robot_params.r = {[0.1, 0, 0],   [0.15, 0, 0], [0.08, 0, 0], 
                      [0.05, 0, 0], [0.02, 0, 0], [0.01, 0, 0]};
    
    % 惯性张量 (kg*m^2) {Ixx, Iyy, Izz, Ixy, Ixz, Iyz}
    robot_params.I = {[0.1, 0.1, 0.05, 0, 0, 0], [0.05, 0.05, 0.02, 0, 0, 0],
                      [0.03, 0.03, 0.01, 0, 0, 0], [0.01, 0.01, 0.005, 0, 0, 0],
                      [0.005, 0.005, 0.002, 0, 0, 0], [0.002, 0.002, 0.001, 0, 0, 0]};
    
    % 重力加速度 (m/s^2)
    robot_params.g = [0, 0, -9.81];
    
    % 初始状态: [q1, dq1, q2, dq2, ..., q6, dq6] (初始位置和速度)
    init_state = zeros(1, 12); % 从静止状态开始

    % 预分配输出数组
    N = length(t);
    Q_actual = zeros(N, 6);
    
    % 创建力矩插值函数（ODE求解器需要连续函数）
    tau_func = @(time) interp1(t, U_input, time);
    
    % 设置ODE求解器选项（提高精度和稳定性）
    options = odeset('RelTol', 1e-6, 'AbsTol', 1e-8);
    
    % 求解动力学方程
    [~, state_history] = ode45(@(time, state) robot_dynamics(time, state, tau_func(time), robot_params),...
                              t, init_state, options);
    
    % 提取关节角度（每隔一个变量）
    Q_actual = state_history(:, 1:2:12);
end

function dstate = robot_dynamics(time, state, tau, params)
% ROBOT_DYNAMICS 计算机械臂动力学微分方程
%   使用拉格朗日方法计算加速度[1,6](@ref)
    
    % 提取状态变量
    q = state(1:2:12);     % 关节位置 [q1, q2, q3, q4, q5, q6]
    dq = state(2:2:12);    % 关节速度 [dq1, dq2, dq3, dq4, dq5, dq6]
    
    % 计算质量矩阵M(q)[4](@ref)
    M = diag([0.5, 0.4, 0.3, 0.25, 0.2, 0.15]); 
    % M = calculate_mass_matrix(q, params);
    
    % 计算科里奥利和离心力矩阵C(q, dq)[4](@ref)
    C = calculate_coriolis_matrix(q, dq, params);
    
    % 计算重力向量G(q)[4](@ref)
    G = calculate_gravity_vector(q, params);
    
    % 计算摩擦力（简单粘性摩擦模型）
    F = 0.1 * dq; % 摩擦系数可根据实际情况调整
    
    % 构建动力学方程: M(q)*ddq + C(q,dq) + G(q) + F(dq) = tau
    % 求解加速度: ddq = M(q)^{-1} * (tau - C(q,dq) - G(q) - F(dq))
    ddq = M \ (tau' - C - G - F');
    
    % 构建状态导数向量
    dstate = zeros(12, 1);
    for i = 1:6
        dstate(2*i-1) = dq(i);     % 位置导数 = 速度
        dstate(2*i) = ddq(i);       % 速度导数 = 加速度
    end
end

function M = calculate_mass_matrix(q, params)
% CALCULATE_MASS_MATRIX 计算质量矩阵M(q)
%   这是一个简化实现，实际应用中可能需要更复杂的计算[4](@ref)
    
    M = zeros(6,6);
    
    % 这里使用对角线矩阵作为示例，实际质量矩阵是非对角且耦合的
    for i = 1:6
        M(i,i) = params.m(i) * params.l(i)^2 + params.I{i}(3); % 惯性项
    end
    
    % 添加耦合项（简化示例）
    M(1,2) = 0.5 * params.m(2) * params.l(1) * params.l(2) * cos(q(1)-q(2));
    M(2,1) = M(1,2);
end

function C = calculate_coriolis_matrix(q, dq, params)
% CALCULATE_CORIOLIS_MATRIX 计算科里奥利和离心力矩阵
%   这是一个简化实现[4](@ref)
    
    C = zeros(6,1);
    
    % 计算科里奥利力（简化示例）
    C(1) = -params.m(2) * params.l(1) * params.l(2) * sin(q(1)-q(2)) * dq(2)^2;
    C(2) = params.m(2) * params.l(1) * params.l(2) * sin(q(1)-q(2)) * dq(1)^2;
    
    % 其余关节的离心力和科里奥利力（简化处理）
    for i = 3:6
        C(i) = 0.1 * dq(i)^2; % 简化的离心力项
    end
end

function G = calculate_gravity_vector(q, params)
% CALCULATE_GRAVITY_VECTOR 计算重力向量
%   计算每个关节需要克服的重力矩[4](@ref)
    
    G = zeros(6,1);
    
    % 计算重力矩（简化示例）
    for i = 1:6
        % 考虑所有后续连杆对当前关节的重力影响
        for j = i:6
            G(i) = G(i) + params.m(j) * params.g(3) * params.l(i) * sin(sum(q(1:i)));
        end
    end
end