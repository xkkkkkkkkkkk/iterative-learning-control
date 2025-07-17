
clc;
clear;
close all;


initial = 3;

tSpin = [0, 10];
[time, state] = ode45(@SystemModel, tSpin, initial);

LINEWIDTH = 1.5;
figure('Name','System State');
plot(time, state, 'LineWidth', LINEWIDTH);
xlabel('time(sec)');
ylabel('');
legend('$$x(t)$$', 'Interpreter', 'latex')



function sys = SystemModel(time, state)

%SYSTEMMODEL  System Model
%   time:running time
%   state:system state

t = time;

x = state(1, 1);

dot_x = -x;

sys = dot_x;

end