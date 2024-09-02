function reward = defReward(scene, rows, cols,R)
% ����scene����ʼ��reward�����ϰ���Ӧ�����븺����
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

 
% ��б���˶��Ľ�������ֵ��Ϊ0.7071
for i=1:rows*cols    
    % �������˶�
    if i-cols-1>0 && reward(i,i-cols-1)~=-100
        reward(i,i-cols-1) = 1/sqrt(2);
    end
    % �������˶�
    if i+cols-1<rows*cols && reward(i,i+cols-1)~=-100
        reward(i,i+cols-1) = 1/sqrt(2);
    end
    % �������˶�
    if i+cols+1<rows*cols && reward(i,i+cols+1)~=-100
        reward(i,i+cols+1) = 1/sqrt(2);
    end
    % �������˶�
    if i-cols+1>0 && i-cols+1<rows*cols && reward(i,i-cols+1)~=-100
        reward(i,i-cols+1) = 1/sqrt(2);
    end
end

% �Ѳ����ڱ��ڵ�8����Ľڵ�Ľ���ֵ��Ϊ-inf
for i=1:rows*cols
    for j=1:rows*cols
        if j~=i-cols  && j~=i+cols  && j~=i-1 && j~=i+1 && j~=i+cols+1 && j~=i+cols-1 && j~=i-cols+1 && j~=i-cols-1
            reward(i,j) = -Inf;
        end
    end
end

% ����ڵ㲻���ڱ߽�ش����ϡ������ϡ��������˶�
for i=1:cols:rows*cols
    for j=1:i+cols
        if j==i+cols-1 || j==i-1 || j==i-cols-1
            reward(i,j) = -Inf;
            reward(j,i) = -Inf;
        end
    end
end
