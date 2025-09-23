% 基于上一次迭代的控制误差进行修正
function Delta_U = ILC_update(Error_prev, t, Kp, Kd)
% 输入: Error_prev - 上一次迭代的误差 (N x 6)
%       t - 时间向量
%       Kp, Kd - 学习增益（可以是标量或6x6对角矩阵）
% 输出: Delta_U - 控制修正量 (N x 6)

    [N, num_joints] = size(Error_prev);
    Delta_U = zeros(N, num_joints);
    dt = t(2) - t(1);

    for j = 1:num_joints
        e_prev = Error_prev(:, j);
        % 计算误差导数（中心差分）
        e_prev_dot = gradient(e_prev, dt);

        % if j == 6
        %     Delta_U(:, j) = 0.001 * e_prev + 0.0001 * e_prev_dot;
        % else

        % PD型学习律: Delta_u(t) = Kp * e_{k-1}(t) + Kd * de_{k-1}/dt(t)
        Delta_U(:, j) = Kp * e_prev + Kd * e_prev_dot;
    end
end
