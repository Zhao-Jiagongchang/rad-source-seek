function [W,X_S_x_old,X_S_y_old,X_S_fai_old,X_S_x_new,X_S_y_new,X_S_fai_new,X_S_x_E,X_S_y_E,ke_sai_of_x]=Pf_estimate(RS,WorldSize,Robot_x,Robot_y,N,W,X_S_x_old,X_S_y_old,X_S_fai_old,X_S_x_new,X_S_y_new,X_S_fai_new)
    RS_x = RS.x;
    RS_y = RS.y;
    RS_fai = RS.fai;
    % figure(1);
    % axis([0, WorldSize, 0, WorldSize]);
    % hold on;
    % plot(X_S_x_old, X_S_y_old, '.', 'MarkerSize', 3, 'Color', 'b'); % 粒子
    % pause(0.6)
    %*** 1.更新步 更新粒子的权重w ***
    fai_of_sensor = sensor(Robot_x, Robot_y, 5);
    fai_of_lambda = lambda(Robot_x, Robot_y, X_S_x_old, X_S_y_old, X_S_fai_old, 5);
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
    std_x = 0.2; % 可以根据需要调整
    std_y = 0.2; % 可以根据需要调整
    std_fai = 0.05; % 可以根据需要调整

    % 给重采样后的粒子添加高斯噪声
    X_S_x_new = X_S_x_new + normrnd(0, std_x, [1, N]);
    X_S_y_new = X_S_y_new + normrnd(0, std_y, [1, N]);
    X_S_fai_new = X_S_fai_new + normrnd(0, std_fai, [1, N]);
    
    % 确保粒子在合理范围内
    X_S_x_new = max(min(X_S_x_new, WorldSize), 0);
    X_S_y_new = max(min(X_S_y_new, WorldSize), 0);
    X_S_fai_new = max(X_S_fai_new, 0); % 假设fai不能为负
    
    %*** 4.估计位置 ****
    X_S_x_E = sum(X_S_x_new, 'omitnan') / N;
    X_S_y_E = sum(X_S_y_new, 'omitnan') / N;
    
    
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
    ke_sai_of_x = sqrt((X_S_x_max - X_S_x_min)^2 + (X_S_y_max - X_S_y_min)^2) / 100;
    ke_sai_of_y = ke_sai_of_x;
    ke_sai_of_fai = sqrt((X_S_fai_max - X_S_fai_min)^2 + (X_S_fai_max - X_S_fai_min)^2) / 200;
    
    % 5.3 高斯函数更新粒子    
    X_S_x_old = normrnd(X_S_x_new, ke_sai_of_x, [1, N]);
    X_S_y_old = normrnd(X_S_y_new, ke_sai_of_y, [1, N]);
    X_S_fai_old = normrnd(X_S_fai_new, ke_sai_of_fai, [1, N]);
    

end