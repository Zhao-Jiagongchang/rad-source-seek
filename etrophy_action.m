function action_index=etrophy_action(Robot_x,Robot_y,WorldSize,ynew,xnew,X_S_x_new,X_S_y_new,X_S_fai_new,u_b,W,epoch_id,R_x_list,R_y_list)
    % 初始化邻居节点
Xneighbour = zeros(1,5);
Yneighbour = zeros(1,5);
    for k = 1:4
        if Robot_x+xnew(k)<0 || Robot_y+ynew(k)<0 || Robot_x+xnew(k)>WorldSize|| Robot_y+ynew(k)>WorldSize
            var(k)=NaN;
            continue
        end
        Xneighbour(k) = Robot_x+xnew(k);
        Yneighbour(k) = Robot_y+ynew(k);
        dist = sqrt((Xneighbour(k)-X_S_x_new).^2 + (Yneighbour(k)-X_S_y_new).^2); % 邻居节点到每个的距离
        conc=(u_b + X_S_fai_new./(dist.^2)).*5;
        prob = W;
        detRate =  1*conc;
        h = detRate;
        Num= 140;
        increasingNbs = 0:Num-1;
        entropy=0;
        for z = 1:Num
            poissonVals = (h.^increasingNbs(z)).*exp(-h)*1./factorial(increasingNbs(z));
            p_d(z) = sum(poissonVals.*prob);  
        end
        p_d = p_d./sum(p_d);%邻居节点概率分布函数
        p_d(p_d < 1e-10) = 1e-10; % 设置最小阈值以避免数值不稳定
        entropy = entropy - sum(p_d .* log2(p_d));
        %entropy = entropy - sum((p_d).*log2(p_d+(p_d==0)));
        var(k) = entropy;
        for z = 1:12
            if epoch_id<=z
                continue
            elseif Xneighbour(k) == R_x_list(epoch_id-z) && Yneighbour(k) == R_y_list(epoch_id-z)
                if z == 1
                    var(k) = var(k)*0.90;
                end
                var(k)= var(k)*0.95;
            end
        end



    end

        [val,ind] = max(var);
        action_index = ind;

end