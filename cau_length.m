% 参数设置
xs = 10;  % 放射源的 x 坐标
ys = 25;  % 放射源的 y 坐标
phis = 1000;  % 距离放射源 1m 处的剂量计数值

% 观测点坐标范围
x_range = 1:50;
y_range = 1:50;

% 本底剂量计数值
mu_b = 0.1;

% 衰减系数和暴露时间
beta_ks = 0.0003;
tau_k = 5;

% 初始化放射值矩阵
lambda_k = zeros(length(x_range), length(y_range));

% 计算放射值
for i = 1:length(x_range)
    for j = 1:length(y_range)
        xi = x_range(i);
        yj = y_range(j);
        diks = sqrt((xi - xs)^2 + (yj - ys)^2)*0.1;
        lambda_k(i, j) = mu_b + (phis / diks^2) * exp(-beta_ks * diks) * tau_k;
    end
end

R=log10(lambda_k)-3;
%%
% 替换Q_bar中的0为-Inf
Q_bar = Q;
Q_bar(Q_bar == 0) = -Inf;

% 找到路径
path_iter = updatePath(Q_bar, startPos, goalPos);

% 计算路径长度
path_iter_sub = [];
[path_iter_sub(:, 1), path_iter_sub(:, 2)] = ind2sub([rows, cols], path_iter);
diffX = diff(path_iter_sub(:, 1));
diffY = diff(path_iter_sub(:, 2));
len = sum(sqrt(diffX.^2 + diffY.^2));  

% 绘制热图和路径
figure;
imagesc(lambda_k);  % 使用lambda_k绘制热图
colorbar;
colormap(jet);  % 使用Jet颜色图
hold on;

% 绘制路径
[path_sub(:, 1), path_sub(:, 2)] = ind2sub([rows, cols], path_iter);
path_target_xy(:, 1) = path_sub(:, 2);
path_target_xy(:, 2) = path_sub(:, 1);
plot(path_target_xy(:, 1), path_target_xy(:, 2), 'g', 'linewidth', 2);  % 路径

% % 绘制起始点和目标点
% plot(startPos(2), startPos(1), 'ys', 'MarkerSize', 10, 'MarkerFaceColor', 'y');  % 起始点
% plot(goalPos(2), goalPos(1), 'ms', 'MarkerSize', 10, 'MarkerFaceColor', 'm');  % 目标点

% 绘制障碍物
scene_temp = scene;
scene_temp(path_iter) = 6;
scene_temp(startPos) = 4;
scene_temp(goalPos) = 5; 
obstacle_ind = find(scene == 2);
[obstacle_sub(:, 1), obstacle_sub(:, 2)] = ind2sub([rows, cols], obstacle_ind);
plot(obstacle_sub(:, 2), obstacle_sub(:, 1), 'ks', 'MarkerSize', 10, 'MarkerFaceColor', 'k');  % 静态障碍物

% 设置图像属性
xlabel('X');
ylabel('Y');
%title('路径规划热图');
axis image;

grid off;  % 移除网格线
hold off;
