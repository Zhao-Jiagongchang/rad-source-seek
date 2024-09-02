clc
clear
close all
rows = 50;
cols = 50;
startPos = 2474;
goalPos = 1210;

% 定义场景
scene = defScene_50(rows, cols, startPos, goalPos);

% 修改指定范围内的方格为绿色
for i = 25:50
    for j = 5:45
        scene(i, j) = 6;  % 6 是绿色的索引，根据你的 colormap 定义
    end
end

figure;
cmap = [1 1 1; ... % 1-白色-空地
    0 0 0; ...     % 2-黑色-静态障碍
    1 0 0; ...     % 3-红色-动态障碍
    1 1 0;...      % 4-黄色-起始点
    1 0 1;...      % 5-品红-目标点
    0 1 0; ...     % 6-绿色-到目标点的规划路径
    0 1 1];        % 7-青色-动态规划的路径

colormap(cmap);

% 绘制场景
image(1.5, 1.5, scene);
grid on;
set(gca, 'GridLineStyle', '-', 'GridColor', 'k', 'LineWidth', 1.8, 'GridAlpha', 1);
set(gca, 'xtick', 1:cols + 1, 'ytick', 1:rows + 1);
axis image;

% 在每个空格中添加编号
% for i = 1:rows
%     for j = 1:cols
%         index = sub2ind([rows, cols], i, j);
%         text(j + 0.5, i + 0.5, num2str(index), 'Color', 'r', 'FontSize', 8, 'HorizontalAlignment', 'center');
%     end
% end

hold on;

