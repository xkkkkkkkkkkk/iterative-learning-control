% 自适应增益示例
function gamma = adaptiveGain(e, de, gamma_base)
    % 误差大时增加增益，接近目标时减小
    error_norm = sqrt(e^2 + 0.1*de^2);
    
    if error_norm > 0.5
        gamma = min(1.2 * gamma_base, 0.8);
    elseif error_norm > 0.1
        gamma = gamma_base;
    else
        gamma = 0.7 * gamma_base;
    end
end