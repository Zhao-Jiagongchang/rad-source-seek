clc
clear
close all
% 参数设置
robot_start = [20, 1];  % 机器人起始位置 (x, y)
source_location = [25, 40];  % 辐射源位置 (x, y)
phi_s = 1000;  % 辐射源强度 (cps)
N = 2000;  % 粒子数量
mu_b = 1;  % 背景辐射 (cps)
tau_k = 5;  % 测量周期 (s)

% 定义网格范围
x = 0:1:50;  % x 轴坐标范围
y = 0:1:50;  % y 轴坐标范围
[X, Y] = meshgrid(x, y);

% 计算每个点的辐射强度 λ_k
dk = sqrt((X - source_location(1)).^2 + (Y - source_location(2)).^2);
lambda_k = (mu_b + (phi_s ./ dk)) * tau_k;

% 对辐射强度取对数
log_lambda_k = log(lambda_k);

% 设置颜色阈值，低于 6 的设置为白色
threshold = 6;

% 绘制等高线图，提高颜色级别的数量
figure;
contourf(X, Y, log_lambda_k, 100, 'LineColor', 'none'); % 50个颜色级别

% 自定义颜色图
cmap = jet(256); % 创建一个 jet 颜色图
cmap(1,:) = [1 1 1]; % 将第一行颜色（对应最低的值）设置为白色
colormap(cmap);

% 设置颜色轴范围，将低于 threshold 的值映射到第一行颜色（白色）
caxis([threshold max(log_lambda_k(:))]);

% 添加色条并标注单位
cb = colorbar;
ylabel(cb, '$$\log(\lambda_k)$$', 'Interpreter', 'latex', 'FontSize', 14);

hold on;

% 标注辐射源位置和机器人起始位置
plot(source_location(1), source_location(2), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
text(source_location(1) + 1, source_location(2), 'Source', 'Color', 'red');
% plot(robot_start(1), robot_start(2), 'bo', 'MarkerSize', 10, 'LineWidth', 2);
% text(robot_start(1) + 1, robot_start(2), 'Robot Start', 'Color', 'blue');

% 图形标签和标题
xlabel('X Position (m)');
ylabel('Y Position (m)');
title('Radiation Intensity Map');
hold off;
