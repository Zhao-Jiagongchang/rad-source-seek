function path = updatePath(Q, start, goal)
    % ��ʼ��ʱ��
    startTime = tic;
    timeoutDuration=1;
    
    % ��ʼ��·��
    path = start;
    move = 0;

    % ѭ������ֱ���ҵ��յ�Goal
    while move ~= goal
        % ����Ƿ�ʱ
        elapsedTime = toc(startTime);
        if elapsedTime > timeoutDuration
            path = NaN;
            return;
        end
        
        % ����㿪ʼ����
        [~, move] = max(Q(start, :));

        % sort()��������������������������Сѭ�������
        step = 2;
        max_steps = length(Q(start, :));  % �������Ӧ��������������
        while ismember(move, path) && step <= max_steps
            [~, x] = sort(Q(start, :), 'descend');
            move = x(step);
            step = step + 1;
        end

        % ������һ��������·����
        path = [path, move];
        start = move;
    end
end
