function [u_total, Useq, k_seq] = advancedILC(Yseq, y_k, x, Useq, k_seq, iter, Ts)
    % 参数持久化存储
    persistent gamma_base Lp Ld alpha Q_filter
    
    % 首次调用初始化
    if isempty(gamma_base)
        gamma_base = 0.5;
        Lp = 0.6; 
        Ld = 0.1;
        alpha = 0.97;
        Q_filter = designFilter(10, Ts); % 10Hz低通
    end
    
    % 自适应增益
    e = Yseq(k_seq) - y_k;
    de = (e - (Yseq(max(1,k_seq-1)) - y_prev)) / Ts;
    gamma = adaptiveGain(e, de, gamma_base);
    
    % 时间步更新
    [u_ff, ~, k_seq, Useq] = ilcUpdateFunction(Yseq, y_k, Useq, k_seq, gamma);
    
    % PD反馈补偿
    u_fb = Kp*e + Kd*de;
    
    % 总控制量
    u_total = u_ff + u_fb;
    
    % 迭代结束后全局优化
    if k_seq == length(Yseq)
        % 1. 应用遗忘因子
        Useq = alpha * Useq;
        
        % 2. 应用低通滤波
        Useq = filtfilt(Q_filter.b, Q_filter.a, Useq);
        
        % 3. 轨迹形状优化 (可选)
        if mod(iter, 5) == 0  % 每5次迭代执行一次
            Useq = optimizeTrajectory(Useq, Yseq);
        end
    end
end