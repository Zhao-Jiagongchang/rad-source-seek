function reward = defReward(scene, rows, cols,R)
% 根据scene，初始化reward矩阵，障碍物应当给与负奖励
reward = zeros(rows*cols);
for i=1:rows*cols
    reward(i,:) = reshape(R,1,rows*cols);   
end
reward_scene = zeros(rows*cols);
for i=1:rows*cols
    reward_scene(i,:) = reshape(scene,1,rows*cols);   
end

reward(reward_scene == 2) = -1000;
reward(reward == Inf) = 1000;

 
% 将斜向运动的奖励矩阵值赋为0.7071
for i=1:rows*cols    
    % 向左上运动
    if i-cols-1>0 && reward(i,i-cols-1)~=-100
        reward(i,i-cols-1) = 1/sqrt(2);
    end
    % 向右上运动
    if i+cols-1<rows*cols && reward(i,i+cols-1)~=-100
        reward(i,i+cols-1) = 1/sqrt(2);
    end
    % 向右下运动
    if i+cols+1<rows*cols && reward(i,i+cols+1)~=-100
        reward(i,i+cols+1) = 1/sqrt(2);
    end
    % 向左下运动
    if i-cols+1>0 && i-cols+1<rows*cols && reward(i,i-cols+1)~=-100
        reward(i,i-cols+1) = 1/sqrt(2);
    end
end

% 把不属于本节点8邻域的节点的奖励值设为-inf
for i=1:rows*cols
    for j=1:rows*cols
        if j~=i-cols  && j~=i+cols  && j~=i-1 && j~=i+1 && j~=i+cols+1 && j~=i+cols-1 && j~=i-cols+1 && j~=i-cols-1
            reward(i,j) = -Inf;
        end
    end
end

% 定义节点不能在边界地带往上、往左上、往右上运动
for i=1:cols:rows*cols
    for j=1:i+cols
        if j==i+cols-1 || j==i-1 || j==i-cols-1
            reward(i,j) = -Inf;
            reward(j,i) = -Inf;
        end
    end
end
