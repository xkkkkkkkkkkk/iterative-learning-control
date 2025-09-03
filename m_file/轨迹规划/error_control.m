%% 误差检查

function [max_err, avg_err] = error_control(bspline, Q_ori, delta_u)
% bspline:拟合的b样条
% Q_ori：原始的型值点集
% delta_u：采样步长

    u_sample = 0:delta_u:1;
    C_u = bspline_eval(bspline, u_sample);% 采样曲线点
    dists = zeros(1, length(u_sample));

    % 计算点到原始线段的最短距离
    for i  = 1:lengh(u_sample)
        point = C_u(i,:);
        min_dist = inf;
        for j = 1:size(Q_ori,1)-1
            d = dist_p2l(point, line_p1, line_p2);
            if d < min_dist
                min_dist = d;
            end
        end
        dists(i) = min_dist;
    end

    max_err = max(dists);
    avg_err = mean(dists);
end

function d = dist_p2l(point, line_p1, line_p2)
    vec1 = linep2 - linep1;
    vec2 = point - line_p1;
    proj_len = dot(vec2,vec1) / norm(vec1);

    if proj_len < 0
        d = norm(point - line_p1);
    elseif proj_len > norm(vec1)
        d = norm(point - line_p2);
    else
        d = abs(vec1(1)*vec2(2) - vec2(1)*vec1(2)) / norm(vec1); % 平行四边形面积/底 = 高
    end
end