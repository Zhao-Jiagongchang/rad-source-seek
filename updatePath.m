function path = updatePath(Q, start, goal)
    % 开始计时器
    startTime = tic;
    timeoutDuration=1;
    
    % 初始化路径
    path = start;
    move = 0;

    % 循环迭代直至找到终点Goal
    while move ~= goal
        % 检查是否超时
        elapsedTime = toc(startTime);
        if elapsedTime > timeoutDuration
            path = NaN;
            return;
        end
        
        % 从起点开始搜索
        [~, move] = max(Q(start, :));

        % sort()排序函数，按降序排序，消除陷入小循环的情况
        step = 2;
        max_steps = length(Q(start, :));  % 最大步数不应超过动作的数量
        while ismember(move, path) && step <= max_steps
            [~, x] = sort(Q(start, :), 'descend');
            move = x(step);
            step = step + 1;
        end

        % 加上下一个动作到路径中
        path = [path, move];
        start = move;
    end
end
