% 加载数据
count_rand_classic = load("count_rand_classic.mat");
count_rand_airf = load("count_rand_airf2.mat");

% 初始化变量
avg_length_classic = zeros(1, length(count_rand_classic.count_list));
avg_length_airf = zeros(1, length(count_rand_airf.count_list));

% 计算经典算法的平均路径长度
for b = 1:length(count_rand_classic.count_list)
    avg_length_classic(b) = mean(count_rand_classic.count_list(1:b));
end

% 计算信息驱动算法的平均路径长度
for b = 1:length(count_rand_airf.count_list)
    avg_length_airf(b) = mean(count_rand_airf.count_list(1:b));
end

% 绘图
figure;
plot(avg_length_classic, '-b', 'LineWidth', 1);
hold on;
plot(avg_length_airf, '-r', 'LineWidth', 1);

% 设置图形属性
ylabel('Average traversal path length of agents/m');
xlabel('Episodes');
legend('\epsilon-greedy', 'Information driving');
title('Comparison of Average Path Lengths');

% % 调整图形导出尺寸为默认窗口大小
% set(gcf, 'PaperPositionMode', 'auto');
% 
% % 导出图形为 PDF 文件
% print(gcf, '-dpdf', 'average_path_lengths.pdf');
exportgraphics(gcf, 'average_path_lengths.pdf', 'ContentType', 'vector', 'Resolution', 300); % 300 DPI 的分辨率

% 关闭 hold
hold off;