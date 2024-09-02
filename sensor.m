function fai = sensor(x,y,t)
% *************************************
%*************传感器模型***************
%放射源的参数  RS = [x_s,y_s,fai_s]'
    RS_x = 25;
    RS_y = 40;
    RS_fai = 1000;%单位cps
    %本底辐射
    u_b = 1;
    %衰减系数
    bei_ta = 0.00003;
    %计算探测器离放射源的距离的平方
    R = sqrt((x - RS_x)^2 + (y - RS_y)^2);
    
    fai_ = (u_b + RS_fai./(R^2) * exp(-1*R*bei_ta)) * t;
    fai = random('Poisson',fai_);
    
    

end