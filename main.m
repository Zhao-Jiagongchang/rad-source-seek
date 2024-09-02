clc
clear
close all
Display = 1;
%% 粒子滤波参数初始化
% *************放射源的设置*******************
% 放射源的参数  RS = [x_s,y_s,fai_s]'
RS.x = 24;
RS.y = 40;
RS.fai = 1000; % 单位cps
% 本底辐射
u_b = 0.01;
WorldSize = 49; % 世界大小
% left, down, right, up
ynew = [0,-1,0,1,0];
xnew = [-1,0,1,0,0];
% ***********机器人设置************
% 初始位置
Robot_x_ini = 19;
Robot_y_ini = 0;
% 运动步长
step = 1;
R_x_list = zeros(1,1000);
R_y_list = zeros(1,1000);
error_list = [];



%% 强化学习参数 %% 场景定义
% 行、列、起始点和目标点
load("reward_rai.mat")

rows = 50;
cols = 50;
startPos = pose2Node(Robot_x_ini,Robot_y_ini,WorldSize);
goalPos = pose2Node(RS.x,RS.y,WorldSize);

% 场景定义
scene = defScene_50(rows,cols,startPos,goalPos);

% 构造奖励矩阵reward
reward = defReward(scene, rows, cols,R);
reward = def_reward_new(reward);
totalStartTime = tic; % 开始计时整个函数
count_list=[];

%% 通过循环迭代得到Q-learning算法的Q值表
% Q矩阵初始化及常量参数
Q = 2*ones(rows*cols);
Q(reward>-2)=10;
Q(reward==100)=300;

gamma = 0.9;
alpha = 0.6;
epsilon = 0.7;
% 初始ε值
len=0;

% 循环相关变量
iter_max = 1000;                   % 循环迭代次数
lens_iter = zeros(1,iter_max);   % 存储每次迭代路径长度
path_cell = cell(0);             % 存储每次迭代的路径

% 循环迭代
for i=1:iter_max
    % *************粒子初始化***************i********

    % 粒子数N
    N = 2000;
    % 粒子初始化
    % 未经过重采样的粒子X_S_old
    X_S_x_old = zeros(1,N);
    X_S_y_old = zeros(1,N);
    X_S_fai_old = zeros(1,N);
    % 重采样后的粒子X_S_new
    X_S_x_new = zeros(1,N);
    X_S_y_new = zeros(1,N);
    X_S_fai_new = zeros(1,N);
    
    
    % 权重
    W = ones(1,N);
    % 粒子初值服从均匀分布采样 unifrnd() 生成一定范围内的随机数
    for ij = 1:N
        X_S_x_old(ij) = unifrnd(0,WorldSize);   
        X_S_y_old(ij) = unifrnd(0,WorldSize);
        X_S_fai_old(ij) = unifrnd(10,20000);
        %W(i) = 1/N;
    end
    Robot_x=Robot_x_ini;
    Robot_y=Robot_y_ini;
    % 减少ε值，随着迭代进行逐渐减小
    epsilon = epsilon * 1;
    iterationStartTime = tic; % 开始计时每次迭代
    % 当前节点状态
    node_now = startPos;
    count=0;
    % 重复运行，直至到达goalPos状态
    Robot_x_list=[];
    Robot_y_list=[];
    ke_sai= 1;
    while true
        
        % 生成当前节点状态的所有可能行动的索引

        count=count+1;
        %[W,X_S_x_old,X_S_y_old,X_S_fai_old,X_S_x_new,X_S_y_new,X_S_fai_new,X_S_x_E,X_S_y_E]=Pf_estimate(RS,WorldSize,Robot_x,Robot_y,N,W,X_S_x_old,X_S_y_old,X_S_fai_old,X_S_x_new,X_S_y_new,X_S_fai_new);
        actions_now = find(reward(node_now,:) >= -4);
        
        % ε-贪婪策略决定选择的行动
        if rand < epsilon
            % 随机选择一个行动
            action_now = actions_now(randi(length(actions_now)));
            %disp('探索')

            %选择熵最大的方向
            %[W,X_S_x_old,X_S_y_old,X_S_fai_old,X_S_x_new,X_S_y_new,X_S_fai_new,X_S_x_E,X_S_y_E,ke_sai]=Pf_estimate(RS,WorldSize,Robot_x,Robot_y,N,W,X_S_x_old,X_S_y_old,X_S_fai_old,X_S_x_new,X_S_y_new,X_S_fai_new);
            % if ke_sai>0.07
            %     [W,X_S_x_old,X_S_y_old,X_S_fai_old,X_S_x_new,X_S_y_new,X_S_fai_new,X_S_x_E,X_S_y_E,ke_sai]=Pf_estimate(RS,WorldSize,Robot_x,Robot_y,N,W,X_S_x_old,X_S_y_old,X_S_fai_old,X_S_x_new,X_S_y_new,X_S_fai_new);
            %     action_index=etrophy_action(Robot_x,Robot_y,WorldSize,ynew,xnew,X_S_x_new,X_S_y_new,X_S_fai_new,u_b,W,count,Robot_x_list,Robot_y_list);
            %     if Robot_x+xnew(action_index) < 0
            %         action_now = pose2Node(Robot_x+xnew(action_index)+1,Robot_y+ynew(action_index),WorldSize);
            % 
            %     elseif Robot_y+ynew(action_index) < 0
            %         action_now = pose2Node(Robot_x+xnew(action_index),Robot_y+ynew(action_index)+1,WorldSize);
            % 
            %     else
            %         action_now = pose2Node(Robot_x+xnew(action_index),Robot_y+ynew(action_index),WorldSize);
            %     end
            % else
            %     action_now = actions_now(randi(length(actions_now)));
            % end
            % 
            
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
        [Robot_x, Robot_y] = node2Pose(node_now,WorldSize);
        Robot_x_list = [Robot_x_list,Robot_x];
        Robot_y_list = [Robot_y_list,Robot_y];

    end
    if Display == 1
        figure(11)
        plot(Robot_x_list, Robot_y_list, '.', 'MarkerSize', 10, 'Color', 'r'); % 机器人轨迹
        xlim([0 WorldSize]);
        ylim([0 WorldSize]);
    end
    count_list = [count_list count];
    

    
     
    if i>100
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
        if i == 500 || i == 550 || i == 600 || i == 700 || i == 800|| i == 900 || i == 1000 || i == iter_max
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
    epsilon = epsilon*0.9995;
    
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
