    Q_bar=Q;
    Q_bar(Q_bar==0)=-Inf;

    %找到路径
    path_iter = updatePath(Q_bar,startPos,goalPos);
    

    % 计算路径长度
    path_iter_sub = [];
    [path_iter_sub(:,1), path_iter_sub(:,2)] = ind2sub([rows, cols], path_iter);
    diffX = diff(path_iter_sub(:,1));
    diffY = diff(path_iter_sub(:,2));
    len = sum(sqrt(diffX.^2 + diffY.^2));  
    

    % 画栅格图
    
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
    