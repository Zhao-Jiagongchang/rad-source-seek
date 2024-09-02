close all
clear all
clc

%% *************放射源的设置*******************

% 放射源的参数  RS = [x_s,y_s,fai_s]'
RS_x = 25;
RS_y = 40;
RS_fai = 100; % 单位cps

RS.x = 25;
RS.y = 40;
RS.fai = 1000; % 单位cps
% 本底辐射
u_b = 1;
WorldSize = 50; % 世界大小
% left, down, right, up
ynew = [0,-1,0,1,0];
xnew = [-1,0,1,0,0];


%% ***********机器人设置************
% 初始位置
Robot_x = 44;
Robot_y = 7;
% 运动步长
step = 1;
R_x_list = zeros(1,1000);
R_y_list = zeros(1,1000);
error_list = [];

%% *************粒子初始化***********************

% 粒子数N
N = 1000;
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
for i = 1:N
    X_S_x_old(i) = unifrnd(0,WorldSize);   
    X_S_y_old(i) = unifrnd(0,WorldSize);
    X_S_fai_old(i) = unifrnd(10,20000);
    %W(i) = 1/N;
end

%% *************粒子滤波*************************

% 初始化可视化窗口
figure(1);
axis([0, WorldSize+10, 0, WorldSize+10]);
hold on;
plot(X_S_x_old, X_S_y_old, '.', 'MarkerSize', 3, 'Color', 'b'); % 粒子



for epoch_id = 1:1000
    epoch_id;
    [W,X_S_x_old,X_S_y_old,X_S_fai_old,X_S_x_new,X_S_y_new,X_S_fai_new,X_S_x_E,X_S_y_E,ke_sai]=Pf_estimate(RS,WorldSize,Robot_x,Robot_y,N,W,X_S_x_old,X_S_y_old,X_S_fai_old,X_S_x_new,X_S_y_new,X_S_fai_new);

    %计算 etrophy
    action_index=etrophy_action(Robot_x,Robot_y,WorldSize,ynew,xnew,X_S_x_new,X_S_y_new,X_S_fai_new,u_b,W,epoch_id,R_x_list,R_y_list);





        % %*** 6.机器人前进***
    % % 6.1 计算方向向量
    % direction_of_x = (X_S_x_E - Robot_x) / sqrt((X_S_x_E - Robot_x)^2 + (X_S_y_E - Robot_y)^2);
    % direction_of_y = (X_S_y_E - Robot_y) / sqrt((X_S_x_E - Robot_x)^2 + (X_S_y_E - Robot_y)^2);
    % % 6.2 机器人位置更新
    % if mod(Robot_y,2) == 0
    %     if Robot_x == 50
    %         Robot_y =Robot_y + 3 * step;
    %     end
    %     Robot_x = Robot_x + 1 * step;
    % 
    % end
    % if mod(Robot_y,2) == 1
    %    if Robot_x == 0
    %         Robot_y =Robot_y + 3 * step;
    %     end
    %     Robot_x = Robot_x - 1 * step;
    % end
    Robot_x = Robot_x+xnew(action_index);
    Robot_y = Robot_y+ynew(action_index);


    % Robot_y = Robot_y + direction_of_y * step;
    R_x_list(epoch_id) = Robot_x;
    R_y_list(epoch_id) = Robot_y;
    
    % error=sqrt((RS_x-Robot_x)^2+(RS_y-Robot_y)^2); %error
    % error_list = [error_list, error];
    % if error<0.5 && epoch_id > 100
    %     break
    % end
    
    % 更新可视化内容
    cla; % 清除当前轴内容
    plot(X_S_x_old, X_S_y_old, '.', 'MarkerSize', 3, 'Color', 'b'); % 粒子
    plot(R_x_list(1:epoch_id), R_y_list(1:epoch_id), '.', 'MarkerSize', 10, 'Color', 'r'); % 机器人轨迹
    plot(X_S_x_E, X_S_y_E, 'o', 'Color', [0.5 0 0.5], 'MarkerSize', 10)
    plot(RS_x, RS_y, '.', 'MarkerSize', 26, 'Color', 'g'); % 放射源位置
    drawnow; % 更新图像


end
hold off;

% figure(3);
% plot(error_list, '-');
% title('Error');