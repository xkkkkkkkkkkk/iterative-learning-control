% 图像处理
[Q_real, line_segment] = extract_boundary('butterfly.png');

% 分段拟合
fitted_curves = {};
for seg = 1:num_segments
    Q_segment = get_segment(Q_real, break_points, seg);
    if length(Q_segment) >=3
        P_ctrl = reverse_caculate(Q_segment);
        fitted_curves{end+1} = struct('ctrl_pts', P_ctrl, 'type', 'spline');
    else
        fitted_curves{end+1} = struct('ctrl_pts', Q_segment, 'type', 'line');
    end
end

% 误差检查与重拟合

for i = 1:length(fitted_curves)
    if strcmp(fitted_curves{i}.type, 'spline')
        [max_err, avg_err] = error_control(fitted_curves{i}, Q_segment, 0.001);
        if max_err > epsilon_allow_max
            [new_break, ~] = max_error_point(Q_segment, fitted_curves{i});
            break_points = sort([break_points; new_break]);

        end
    end
end

% 可视化结果

figure;
plot(Q_real(:,1), Q_real(:,2), 'ko');
hold on;
% for a in fitted_curves % 有问题
    if strump(curve.type, 'spline')
        u = linespace(0,1,100);
        plot(bspline_eval(curve.ctrl_pts,u ));
    else
        plot(curve.points(:,1), curve.points(:,2), 'r-');
    end
% end
title('拟合结果');
legend('原始点', 'B样条', '直线段');