clc
clear
close all

files = dir('D:\Users\crcrisoft\Desktop\TraceDataTool_example\对比\*.mat');

for i = 1:length(files)
    filePath = fullfile(files(i).folder, files(i).name);
    load(filePath);  
end

% 时间对比(横坐标比对)
figure;
hold on
plot(pos_cmd_n(:,1));
plot(pos_cmd_m(:,1));
plot(pos_cmd_l(:,1));
plot(pos_cmd_b(:,1));
title('pos cmd,横坐标对比时间');
legend('none','middle','large','bspline');


% 位置对比(cmd&act)
figure
subplot(4,1,1);
hold on 
plot(pos_cmd_b(:,1));
plot(pos_act_b(:,1));
title('bspline pos');
legend('cmd','act')

subplot(4,1,2);
hold on
plot(pos_cmd_n(:,1));
plot(pos_act_n(:,1));
title('none pos');
legend('cmd','act')

subplot(4,1,3);
hold on
plot(pos_cmd_m(:,1));
plot(pos_act_m(:,1));
title('middle pos');
legend('cmd','act')

subplot(4,1,4);
hold on
plot(pos_cmd_l(:,1));
plot(pos_act_l(:,1));
title('large pos');
legend('cmd','act')

% 位置对比（err）
pos_err_b = pos_cmd_b - pos_act_b;
pos_err_n = pos_cmd_n - pos_act_n;
pos_err_m = pos_cmd_m - pos_act_m;
pos_err_l = pos_cmd_l - pos_act_l;
figure
subplot(2,2,1);
plot(pos_err_b(:,1));
title('bspline pos err');
subplot(2,2,2);
plot(pos_err_n(:,1));
title('none pos err');
subplot(2,2,3);
plot(pos_err_m(:,1));
title('middle pos err');
subplot(2,2,4);
plot(pos_err_l(:,1));
title('large pos err');

% 速度对比(cmd&act)
figure
subplot(4,1,1);
hold on 
plot(vel_cmd_b(:,1));
plot(vel_act_b(:,1));
title('bspline vel');
legend('cmd','act')

subplot(4,1,2);
hold on
plot(vel_cmd_n(:,1));
plot(vel_act_n(:,1));
title('none vel');
legend('cmd','act')

subplot(4,1,3);
hold on
plot(vel_cmd_m(:,1));
plot(vel_act_m(:,1));
title('middle vel');
legend('cmd','act')

subplot(4,1,4);
hold on
plot(vel_cmd_l(:,1));
plot(vel_act_l(:,1));
title('large vel');
legend('cmd','act')

% 速度对比（err）
vel_err_b = vel_cmd_b - vel_act_b;
vel_err_n = vel_cmd_n - vel_act_n;
vel_err_m = vel_cmd_m - vel_act_m;
vel_err_l = vel_cmd_l - vel_act_l;
figure
subplot(2,2,1);
plot(vel_err_b(:,1));
title('bspline vel err');
subplot(2,2,2);
plot(vel_err_n(:,1));
title('none vel err');
subplot(2,2,3);
plot(vel_err_m(:,1));
title('middle vel err');
subplot(2,2,4);
plot(vel_err_l(:,1));
title('large vel err');

% 力矩对比(cmd&act)
figure
subplot(4,1,1);
hold on 
plot(trq_cmd_b(:,1));
plot(trq_act_b(:,1));
title('bspline trq');
legend('cmd','act')

subplot(4,1,2);
hold on
plot(trq_cmd_n(:,1));
plot(trq_act_n(:,1));
title('none trq');
legend('cmd','act')

subplot(4,1,3);
hold on
plot(trq_cmd_m(:,1));
plot(trq_act_m(:,1));
title('middle trq');
legend('cmd','act')

subplot(4,1,4);
hold on
plot(trq_cmd_l(:,1));
plot(trq_act_l(:,1));
title('large trq');
legend('cmd','act')

% 力矩对比（err）
trq_err_b = trq_cmd_b - trq_act_b;
trq_err_n = trq_cmd_n - trq_act_n;
trq_err_m = trq_cmd_m - trq_act_m;
trq_err_l = trq_cmd_l - trq_act_l;
figure
subplot(2,2,1);
plot(vel_err_b(:,1));
title('bspline trq err');
subplot(2,2,2);
plot(vel_err_n(:,1));
title('none trq err');
subplot(2,2,3);
plot(vel_err_m(:,1));
title('middle trq err');
subplot(2,2,4);
plot(vel_err_l(:,1));
title('large trq err');