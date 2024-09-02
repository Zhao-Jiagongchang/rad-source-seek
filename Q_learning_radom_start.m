clc
clear
close all

%% 场景定义
% 行、列、起始点和目标点
load("reward_rai.mat")
rows = 50;
cols = 50;
startPos = 1000;
goalPos = 1210;

% 场景定义
scene = defScene_50(rows,cols,startPos,goalPos);

% 构造奖励矩阵reward
reward = defReward(scene, rows, cols,R);
reward = def_reward_new(reward);
totalStartTime = tic; % 开始计时整个函数

%% 通过循环迭代得到Q-learning算法的Q值表
% Q矩阵初始化及常量参数
Q = zeros(rows*cols);
Q(reward>-2)=10;
Q(reward==100)=300;

gamma = 0.9;
alpha = 0.6;
epsilon = 0.7;
% 初始ε值
len=0;


% 循环相关变量
iter_max = 7000;                   % 循环迭代次数
lens_iter = zeros(1,iter_max);   % 存储每次迭代路径长度
path_cell = cell(0);             % 存储每次迭代的路径
count_list=[];

% 循环迭代
for i=1:iter_max
    robot_x= randi([1, 45]);
    robot_y= randi([1, 20]);
    startPos = pose2Node(robot_x,robot_y,49);
    fprintf('第 %d 次迭代执行起点:( %d, %d) %d \n', i,robot_x,robot_y,startPos);

    % 减少ε值，随着迭代进行逐渐减小
    epsilon = epsilon * 1;
    iterationStartTime = tic; % 开始计时每次迭代
    % 当前节点状态
    node_now = startPos;
    count=0;
    % 重复运行，直至到达goalPos状态
    while true
        
        % 生成当前节点状态的所有可能行动的索引
        count=count+1;
        
        actions_now = find(reward(node_now,:) >= -4);
        
        % ε-贪婪策略决定选择的行动
        if rand < epsilon
            % 随机选择一个行动
            action_now = actions_now(randi(length(actions_now)));
            %disp('探索')
        else
            % 选择最优行动（最大Q值）
            [~, idx] = max(Q(node_now, actions_now));
            action_now = actions_now(idx);
            %disp('利用')
        end
        
        % 找到下一个状态所有可能的动作
        actions_next = find(reward(action_now,:) >= -4);
        
        % 在下一个状态的这些所有可能动作中，找到最大的Q值
        q_max = max(Q(action_now, actions_next));
        
        % 更新Q值表
        Q(node_now,action_now) = (1-alpha) * Q(node_now,action_now) + alpha * (reward(node_now,action_now) + gamma * q_max);
        
        % 检查是否到达终点
         if(node_now == goalPos||count==1200)
            break;
        end
        
        % 把下一状态设置为当前状态
        node_now = action_now;
    end
    
    count_list = [count_list count];
     
    if i>2
        Q_bar=Q;
        Q_bar(Q_bar==0)=-Inf;
    
        %找到路径
        path_iter = updatePath(Q_bar,startPos,goalPos);
        path_cell{1,i} = path_iter;
    
        % 计算路径长度
        path_iter_sub = [];
        [path_iter_sub(:,1), path_iter_sub(:,2)] = ind2sub([rows, cols], path_iter);
        diffX = diff(path_iter_sub(:,1));
        diffY = diff(path_iter_sub(:,2));
        len = sum(sqrt(diffX.^2 + diffY.^2));  
        lens_iter(i) = len;
    
        % 画栅格图
        if i == 500 || i == 1000 || i == 1500 || i == 2000 || i == 3000|| i == 3500 || i == 4000 || i == iter_max-1 || i == iter_max-2 || i == iter_max-3 || i == iter_max-4 || i == iter_max-5 || i == iter_max
            figure;
            cmap = [1 1 1; ...       % 1-白色-空地
                0 0 0; ...           % 2-黑色-静态障碍
                1 0 0; ...           % 3-红色-动态障碍
                1 1 0;...            % 4-黄色-起始点
                1 0 1;...            % 5-品红-目标点
                0 1 0; ...           % 6-绿色-到目标点的规划路径
                0 1 1];              % 7-青色-动态规划的路径
    
            % 构建颜色MAP图
            colormap(cmap);
            % 画栅格图
            scene_temp = scene;
            scene_temp(path_iter) = 6;
            scene_temp(startPos) = 4;
            scene_temp(goalPos) = 5; 
            image(1.5,1.5,scene_temp);
            grid on;  % 启用网格线
            set(gca, 'GridLineStyle', '-', 'GridColor', 'k', 'LineWidth', 1.8, 'GridAlpha', 1);
            set(gca,'xtick',1:cols+1,'ytick',1:rows+1);
            axis image;
            hold on
    
            % 画折线段
            path_sub = [];
            path_target_xy = [];
            [path_sub(:,1),path_sub(:,2)] = ind2sub([rows, cols],path_iter);
            path_target_xy(:,1) = path_sub(:,2)+0.5;
            path_target_xy(:,2) = path_sub(:,1)+0.5;
            plot(path_target_xy(:,1),path_target_xy(:,2),'r','linewidth',2)
        end
    end
    iterationElapsedTime = toc(iterationStartTime);
    fprintf('第 %d 次迭代执行时间: %f 秒，agent 探索 %d 次，最短路径长度 %f \n', i, iterationElapsedTime,count,len);
end
%% 画适应度迭代图
figure
hold on
grid on
% 主体图形绘制
plot(1:iter_max, lens_iter,'LineWidth', 3,  'Color', 'b');
% 坐标轴
set(gca,'LineWidth',2.5,'FontName', 'Times New Roman')
hXLabel = xlabel('episodes');
hYLabel = ylabel('length of path/m');
% 修改刻度标签字体和字号
set(gca, 'FontSize', 16),...
    set([hXLabel, hYLabel], 'FontName',  'simsun')
set([hXLabel, hYLabel], 'FontSize', 16)
% disp('最短路径:')
% disp(lens_iter(iter_max))
fprintf('最短路径: %.2f\n', lens_iter(iter_max));
totalElapsedTime = toc(totalStartTime)/60;
fprintf('updatePath 函数执行总时间: %f 分钟\n', totalElapsedTime);
figure;
plot(count_list)
