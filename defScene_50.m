function scene = defScene_50(rows,cols,startPos,goalPos)
 
% 新建一个全为1的scene矩阵
scene = ones(rows,cols);  

% 障碍物
% %scene(5:6,5:12) = 2;        % 放大并平滑位置
% scene(13:14,7:12) = 2;      
% scene(10:12,22:30) = 2;     
% scene(5:6,26:34) = 2;       
% scene(17:20,28:36) = 2;     
% scene(20:21,4:13) = 2;      
% scene(35:40,38:40) = 2;     
% scene(7:22,42:45) = 2;      
% scene(7:22,14:16) = 2;      
% scene(22:50,4:6) = 2;       
% % scene(22,6:14) = 2;                        
% % scene(12:32,20:22) = 2;        
% %scene(34,26:34) = 2;            
% scene(22:34,26:28) = 2;     
% %scene(22,26:34) = 2;                  

% 起点和终点
scene(goalPos) = 5;  % 指定终点坐标 
scene(startPos) = 1; % 考虑到后续的奖励reward，这里设为1
