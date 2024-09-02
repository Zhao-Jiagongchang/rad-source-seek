function reward = def_reward_new(R)
    reward = R;
    % 将障碍物位置的奖励设置为-100
     reward(reward == 2) = -100;

    % 将目标位置的奖励设置为100
    reward(reward == 5) = 1000;
    % 将上下左右移动设置为0
    reward(reward == 1) = -1;    
    % 将斜向运动设置为-inf
    %reward(reward == 1/sqrt(2)) = -1*2/sqrt(2);
    reward(reward == 1/sqrt(2)) = -Inf;
end
  