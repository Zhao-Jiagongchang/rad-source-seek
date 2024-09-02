close all
clear all
clc

%% *************放射源的设置*******************

% 放射源的参数  RS = [x_s,y_s,fai_s]'
RS_x = 25;
RS_y = 40;
RS_fai = 1000; % 单位cps

% 本底辐射
u_b = 0.01;
WorldSize = 50; % 世界大小

%% ***********机器人设置************
% 初始位置
Robot_x = 20;
Robot_y = 0;
% 运动步长
step = 1;
R_x_list = zeros(1,1000);
R_y_list = zeros(1,1000);
error_list = [];
Neff_list=[];
Neff =0;

%% *************粒子初始化***********************

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
for i = 1:N
    X_S_x_old(i) = unifrnd(0,WorldSize);   
    X_S_y_old(i) = unifrnd(0,WorldSize);
    X_S_fai_old(i) = unifrnd(10,20000);
    %W(i) = 1/N;
end

%% *************粒子滤波*************************

% 初始化可视化窗口
figure(1);
axis([0, WorldSize, 0, WorldSize]);
hold on;
%plot(X_S_x_old, X_S_y_old, '.', 'MarkerSize', 3, 'Color', 'b'); % 粒子
%pause(0.6)

for epoch_id = 1:130
    epoch_id
    %*** 1.更新步 更新粒子的权重w ***
    fai_of_sensor = sensor(Robot_x, Robot_y, 3)
    fai_of_lambda = lambda(Robot_x, Robot_y, X_S_x_old, X_S_y_old, X_S_fai_old, 3);
    pdf_of_poiss = poisspdf(fai_of_sensor, fai_of_lambda);
    W = W.*pdf_of_poiss;
    % for id_of_N = 1:N
    %     fai_of_lambda = lambda(Robot_x, Robot_y, X_S_x_old(id_of_N), X_S_y_old(id_of_N), X_S_fai_old(id_of_N), 5);
    % 
    %     % 计算似然概率密度值
    %     pdf_of_poiss = poisspdf(fai_of_sensor, fai_of_lambda);
    %     W(id_of_N) = W(id_of_N) * pdf_of_poiss;
    % end
    %*** 2.权重进行归一化 ***
    W = W / sum(W, 'omitnan');

    Neff = 1 / sum(W .^ 2);
    Neff_list = [Neff_list,Neff]; 
    %if Neff<500 || isnan(Neff)  
        % *** 3.重采样 ***
        % 3.1 构建累计分布
        c = zeros(1, N); % 累积分布函数
        c(1) = W(1);
        u = zeros(1, N);
        u(1) = unifrnd(0, 1/N);
        for id_of_N = 2:N
            c(id_of_N) = c(id_of_N - 1) + W(id_of_N);
            u(id_of_N) = (id_of_N - 1) / N + u(1);
        end
        
        % 3.2 比较
        for i = 1:N
            for j = 1:N
                if u(i) < c(j)
                    X_S_x_new(i) = X_S_x_old(j);
                    X_S_y_new(i) = X_S_y_old(j);
                    X_S_fai_new(i) = X_S_fai_old(j);
                    break;
                end
            end
            % 权重再次归一化
            W(i) = 1 / N;             
        end
        
        % *** 在重采样后添加随机扰动 ***
        % 设置高斯噪声的标准差
        std_x = 0.5; % 可以根据需要调整
        std_y = 0.5; % 可以根据需要调整
        std_fai = 0.05; % 可以根据需要调整

        % 给重采样后的粒子添加高斯噪声
        X_S_x_new = X_S_x_new + normrnd(0, std_x, [1, N]);
        X_S_y_new = X_S_y_new + normrnd(0, std_y, [1, N]);
        X_S_fai_new = X_S_fai_new + normrnd(0, std_fai, [1, N]);
        % 
         % 确保粒子在合理范围内
        X_S_x_new = max(min(X_S_x_new, WorldSize), 0);
        X_S_y_new = max(min(X_S_y_new, WorldSize), 0);
        X_S_fai_new = max(X_S_fai_new, 0); % 假设fai不能为负
    %end
    %*** 4.估计位置 ****
    X_S_x_E = sum(X_S_x_new, 'omitnan') / N;
    X_S_y_E = sum(X_S_y_new, 'omitnan') / N;

    Neff = 1 / sum(W .^ 2);
    
    %*** 5.更新粒子***
    % 5.1 计算粒子的最大、最小值
    % 最大值定义
    X_S_x_max = max(X_S_x_new);
    X_S_y_max = max(X_S_y_new);
    X_S_fai_max = max(X_S_fai_new);
    % 最小值定义
    X_S_x_min = min(X_S_x_new);
    X_S_y_min = min(X_S_y_new);
    X_S_fai_min = 0;
    
    % 5.2 计算标准差
    ke_sai_of_x = sqrt((X_S_x_max - X_S_x_min)^2 + (X_S_y_max - X_S_y_min)^2) / 50;
    ke_sai_of_y = ke_sai_of_x;
    ke_sai_of_fai = sqrt((X_S_fai_max - X_S_fai_min)^2 + (X_S_fai_max - X_S_fai_min)^2) / 100;
    
    % 5.3 高斯函数更新粒子    
    X_S_x_old = normrnd(X_S_x_new, ke_sai_of_x, [1, N]);
    X_S_y_old = normrnd(X_S_y_new, ke_sai_of_y, [1, N]);
    X_S_fai_old = normrnd(X_S_fai_new, ke_sai_of_fai, [1, N]);
    
    %*** 6.机器人前进***
    % 6.1 计算方向向量
    direction_of_x = (X_S_x_E - Robot_x) / sqrt((X_S_x_E - Robot_x)^2 + (X_S_y_E - Robot_y)^2);
    direction_of_y = (X_S_y_E - Robot_y) / sqrt((X_S_x_E - Robot_x)^2 + (X_S_y_E - Robot_y)^2);
    % 6.2 机器人位置更新
    Robot_x = Robot_x + direction_of_x * step;
    Robot_y = Robot_y + direction_of_y * step;
    R_x_list(epoch_id) = Robot_x;
    R_y_list(epoch_id) = Robot_y;
    error=sqrt((RS_x-Robot_x)^2+(RS_y-Robot_y)^2); %error
    error_list = [error_list, error];
    if error<0.5 && epoch_id > 100
        break
    end
    
    % 更新可视化内容
    cla; % 清除当前轴内容
    %plot(RS_x, RS_y, '.', 'MarkerSize', 26, 'Color', 'g'); % 放射源位置
    plot(X_S_x_new, X_S_y_new, '.', 'MarkerSize', 3, 'Color', 'b'); % 粒子
    plot(R_x_list(1:epoch_id), R_y_list(1:epoch_id), '.', 'MarkerSize', 10, 'Color', 'r'); % 机器人轨迹
    plot(X_S_x_E, X_S_y_E, 'o', 'Color', [0.5 0 0.5], 'MarkerSize', 10)%估计位置
    plot(RS_x, RS_y, '.', 'MarkerSize', 26, 'Color', 'g'); % 放射源位置
    legend('particle','robot path','estimate source','real source')
    drawnow; % 更新图像
    %pause(0.05)
end
hold off;

figure(3);
plot(error_list, '-',LineWidth=1);
title('Error');
ylabel('length/m')
xlabel('time/s')
legend('error')