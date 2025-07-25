function tau_g = test_fc(g, DH_params, m, cm_pos, I, fb, fc)

g = [0, 0, 9.81];

tau_g = inverseDynamics(zeros(6,1),zeros(6,1),zeros(6,1), g, DH_params, m, cm_pos, I, fb, fc);
fprintf('\n重力矩\n');
disp(tau_g);
    for i = 1:6
        if abs(tau_g(i)) < fc(i)
            fprintf('关节%.2f重力矩小于静摩擦，无法启动\n',1i);
        else
            fprintf('一切正常');
        end
    end
end