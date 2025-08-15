function dws = CalDWbyq4Sleap(StartQuat, EndQuat,theta,tu,du)
    % 通过四元数来计算角加速度  这里针对四元数  这里的角速度是关于位置的，不是关于时间的  位置为0到1的参数
   [ws,a]=CalWbyq4Sleap(StartQuat, EndQuat,theta,tu);
   [wsa,a]=CalWbyq4Sleap(StartQuat, EndQuat,theta,tu+du);
   dws =(wsa-ws)/du;
end