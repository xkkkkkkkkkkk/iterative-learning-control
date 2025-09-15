clc;
clear;
close all;

% 参数设置
theta_circle = 51;          % 限定角度（度）
epsilon_allowmax = 0.05;    % 允许最大误差（mm）
autu_threshold = 0.02;
%% 图像读取 Q = [x1, y1; x2, y2; ...];
Q = extract_boundary('half_butterfly.png');
break_points = [1, size(Q, 1)];

%% 三误差判断
for i = 2:size(Q, 1)-1
    vec1 = Q(i, :) - Q(i-1, :);
    vec2 = Q(i+1, :) - Q(i, :);

    % % 转角断点判断
    cos = dot(vec1, vec2) / (norm(vec1) * norm(vec2));
    % disp(cos);
    theta_i = acosd(cos);
    % disp(theta_i);
    if theta_i > theta_circle
        break_points = [break_points, i];
        continue; % 跳过后续判断
    end
    
    % 弓高误差判断
    l_i = norm(vec1);
    l_i1 = norm(vec2);
    r = (l_i + l_i1) / (4 * sind(theta_i/2)); % 近似半径
    % disp(r)
    delta_l = r * (1 - cosd(theta_i/2));      % 弓高误差

    % 筛选转角为0的点跳过弓高误差
    if theta_i < 0.1
        delta_l = 0;
    end
    % disp(delta_l);
    if delta_l > epsilon_allowmax
        break_points = [break_points, i];
        continue;
    end
    
    % 凹凸性检测
    % cross_dot = vec1(1)*vec2(2) - vec1(2)*vec2(1); % 叉积Z分量
    % disp(cross_dot);
    % if i > 2
    %     vec_prev = Q(i-1, :) - Q(i-2, :);
    %     vec_curr = Q(i, :) - Q(i-1, :);
    %     cross_prev = vec_prev(1)*vec_curr(2) - vec_prev(2)*vec_curr(1);
    %     % 阈值判断，排除噪声干扰
    %     if abs(cross_dot) < autu_threshold && sign(cross_prev) ~= sign(cross_dot)
    %         cross_dot = - cross_dot;
    %     end
    % 
    %     if sign(cross_prev) ~= sign(cross_dot) && sign(cross_prev) ~= 0 && sign(cross_dot) ~= 0
    %         break_points = [break_points, i];
    %     end
    % end
end

% 断点排序去重
break_points = unique(sort(break_points));
 disp(break_points);


%% 分段拟合
fitted_curves = {};
for seg = 1:length(break_points)-1
    start_idx = break_points(seg);
    end_idx = break_points(seg+1);
    Q_segment = Q(start_idx:end_idx, :);
    m = size(Q_segment, 1); % 当前段内型值点数量
    
    if m >= 3
        % 生成三次B样条
        try
            % B样条拟合函数（局部定义）
            [ctrl_pts] = reverse_caculate(Q_segment);
            fitted_curves{end+1} = struct('type', 'spline', 'ctrl_pts', ctrl_pts, 'points', Q_segment);
        catch
            fitted_curves{end+1} = struct('type', 'line', 'points', Q_segment);
        end
    else
        fitted_curves{end+1} = struct('type', 'line', 'points', Q_segment);
    end
end

% 计算拟合误差并分断
new_break_points = [];
for idx = 1:length(fitted_curves)
    curve = fitted_curves{idx};
    if strcmp(curve.type, 'spline')
        [max_err, err_points] = calculate_error(curve.ctrl_pts, curve.points);
        if max_err > epsilon_allowmax
            [~, max_idx] = max(err_points);
            % 找到全局索引（需要根据曲线段确定）
            %global_idx = find(Q(:,1) == curve.points(max_idx,1) & Q(:,2) == curve.points(max_idx,2));
            global_idx = break_points(idx) + max_idx -1;
            if iscolumn(new_break_points);
                new_break_points = new_break_points';
            end
            if iscolumn(global_idx)
                global_idx = global_idx';
            end
            new_break_points = [new_break_points, global_idx];
        end
    end
end

% 添加新分断点并重新分段（只进行一次）
if ~isempty(new_break_points)
    break_points = unique([break_points, new_break_points]);
    fitted_curves = {};
    for seg = 1:length(break_points)-1
        start_idx = break_points(seg);
        end_idx = break_points(seg+1);
        Q_segment = Q(start_idx:end_idx, :);
        m = size(Q_segment, 1);
        
        if m >= 3
            try
                [ctrl_pts] = reverse_caculate(Q_segment);
                fitted_curves{end+1} = struct('type', 'spline', 'ctrl_pts', ctrl_pts, 'points', Q_segment);
            catch
                fitted_curves{end+1} = struct('type', 'line', 'points', Q_segment);
            end
        else
            fitted_curves{end+1} = struct('type', 'line', 'points', Q_segment);
        end
    end
end

% 输出结果
disp('拟合完成：');
for idx = 1:length(fitted_curves)
    curve = fitted_curves{idx};
    if strcmp(curve.type, 'spline')
        disp(['段', num2str(idx), ': B样条曲线，控制点个数: ', num2str(size(curve.ctrl_pts,1))]);
    else
        disp(['段', num2str(idx), ': 直线段，点数: ', num2str(size(curve.points,1))]);
    end
end

%% 可视化结果
figure;
plot(Q(:,1), Q(:,2)); hold on;
for idx = 1:length(fitted_curves)
    curve = fitted_curves{idx};
    if strcmp(curve.type, 'spline')
        % 生成B样条曲线上的点
        u = linspace(0, 1, 100);
        pts = bspline_eval(curve.ctrl_pts, u);
        % 绘制B样条曲线，使用蓝色实线
        plot(pts(:,1), pts(:,2), 'b-', 'LineWidth', 2, 'DisplayName', 'B样条段');
    else
        % 绘制直线段，使用红色实线
        plot(curve.points(:,1), curve.points(:,2), 'r-', 'LineWidth', 2, 'DisplayName', '直线段');
    end
end

% 标记断点位置（可选）
plot(Q(break_points, 1), Q(break_points, 2), 'gs', 'MarkerSize', 8, 'MarkerFaceColor', 'g', 'DisplayName', '断点');

hold off; % 释放图形
title('B样条拟合结果');
legend('show'); % 显示图例
axis equal;
xlim([min(Q(:,1))-1, max(Q(:,1))+1]);
ylim([min(Q(:,2))-1, max(Q(:,2))+1]);
grid on;

% B样条插值拟合
% 输入：分段型值点集
% 输出：控制点
function [P] = reverse_caculate(Q)
    n = size(Q,1) - 1;
    k = 3;
    chord_length = sqrt(sum(diff(Q).^2, 2));
    u = [0; cumsum(chords)] / sum(chords);
    m = n + k +1;
    U = [zeros(1,k); linspace(0,1,m-2*k)]; % 累计弦长参数化
    
    A = zeros(n+1,n+1);

    for i = 0:n
        
        span = find_span(u(i+1), U, n, k);
        N = basis_functions(span, u(i+1), U, k);
        for j = 0:k
            A(i+1, span-k+j+1) = N(j+1);
        end
    end
    % 添加边界条件
    A(1,1) = 1;
    A(end,end) = 1;
    P = A \ Q;
end

% 辅助函数：B样条求值
function pts = bspline_eval(ctrl_pts, u)
    n = size(ctrl_pts,1) - 1;
    k = 3;
    U = linspace(0,1,n+k+2); 
    for i = 1:length(u)
        span = find_span(u(i), U, n, k);
        N = basis_functions(span, u(i), U, k);
        pt = zeros(1,2);
        for j = 0:k
            pt = pt + N(j+1) * ctrl_pts(span-k+j+1, :);
        end
        pts(i,:) = pt;
    end
end

% 辅助函数：查找参数区间
function span = find_span(u, U, n, k)
    if u >= U(n+1)
        span = n;
    elseif u <= U(k+1)
        span = k;
    else
        low = k;
        high = n+1;
        while u < U(low) || u >= U(high)
            mid = floor((low+high)/2);
            if u < U(mid)
                high = mid;
            else
                low = mid;
            end
        end
        span = low;
    end
end

% 辅助函数：计算基函数
function N = basis_functions(span, u, U, k)
    N = zeros(1,k+1);
    left = zeros(1,k+1);
    right = zeros(1,k+1);
    N(1) = 1;
    for j = 1:k
        left(j+1) = u - U(span+1-j);
        right(j+1) = U(span+j) - u;
        saved = 0;
        for r = 0:j-1
            temp = N(r+1) / (right(r+2) + left(j-r+1));
            N(r+1) = saved + right(r+2) * temp;
            saved = left(j-r+1) * temp;
        end
        N(j+1) = saved;
    end
end

% 辅助函数：计算误差
function [max_err, err_points] = calculate_error(ctrl_pts, Q_segment)
    u = linspace(0,1,100);
    curve_pts = bspline_eval(ctrl_pts, u);
    err_points = zeros(size(Q_segment,1),1);
    for i = 1:size(Q_segment,1)
        point = Q_segment(i,:);
        dists = sqrt(sum((curve_pts - point).^2,2));
        err_points(i) = min(dists);
    end
    max_err = max(err_points);
end