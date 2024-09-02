function fai = lambda(x,y,RS_x,RS_y,RS_fai,t)
% *************************************
%*************辐射模型***************
%放射源的参数  RS = [x_s,y_s,fai_s]'
   
    %本底辐射
    u_b = 1;
    %衰减系数
    bei_ta = 0.00003;
    %计算探测器离放射源的距离的平方
    R = sqrt((x - RS_x).^2 + (y - RS_y).^2);
    %R = sqrt((x - RS_x)^2 + (y - RS_y)^2);
    
    %fai = (u_b + RS_fai/(R^2) * exp(-1*R*bei_ta)) * t;
    fai = (u_b + RS_fai./(R.^2).* exp(-1*R*bei_ta)) * t;

end
