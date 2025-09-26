% 基于上一次迭代的控制误差进行修正
function Delta_U = ILC_update(Error_prev, t, Kp, Kd)
% 输入: Error_prev - 上一次迭代的误差 (N x 6)
%       t - 时间向量
%       Kp, Kd - 学习增益（可以是标量或6x6对角矩阵）
% 输出: Delta_U - 控制修正量 (N x 6)
alpha = 0.3;

    [N, num_joints] = size(Error_prev);
    Delta_U = zeros(N, num_joints);
    dt = t(2) - t(1);

    Error_filtered = zeros(N, num_joints);
    for j = 1:num_joints
        e_prev = Error_prev(:, j);
        e_filtered = zeros(N, 1);
        e_filtered(1) = e_prev(1);

        for i = 2:N
            e_filtered(i) = alpha*e_prev(i) + (1 - alpha)*e_filtered(i - 1);
        end
        Error_filtered(:, j) = e_filtered;
        e_filtered_dot = gradient(e_filtered, dt);

        % if j == 6
        %     Delta_U(:, j) = 0.001 * e_prev + 0.0001 * e_prev_dot;
        % else

        % PD型学习律: Delta_u(t) = Kp * e_{k-1}(t) + Kd * de_{k-1}/dt(t)
        Delta_U(:, j) = Kp * e_prev + Kd * e_filtered_dot;
    end
end
