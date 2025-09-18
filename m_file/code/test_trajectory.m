% 示例1: 点对点轨迹
% t = linspace(0, 5, 100);
% startPose = [0, 0, 0, 0, 0, 0];
% endPose = [1, 1, 1, pi/4, pi/6, pi/2];
% 
% [pos, euler, vel, acc, omega, alpha] = GenerateDesiredTrajectory(t, 'pointToPoint', startPose, endPose);

% 示例2: 贝塞尔曲线轨迹
controlPoints = [0, 0, 0; 
                 1, 2, 0; 
                 3, 3, 1; 
                 4, 1, 2];
             
% 自定义姿态函数
attitudeFunc = @(s) [0, 0, s*pi/2];

[pos, euler, vel, acc, omega, alpha] = GenerateDesiredTrajectory(t, 'bezier', controlPoints, attitudeFunc);

% 可视化轨迹
figure;
plot3(pos(:,1), pos(:,2), pos(:,3), 'b-', 'LineWidth', 2);
hold on;
plot3(controlPoints(:,1), controlPoints(:,2), controlPoints(:,3), 'ro--');
grid on;
xlabel('X'); ylabel('Y'); zlabel('Z');
legend('轨迹', '控制点');
title('贝塞尔曲线轨迹');