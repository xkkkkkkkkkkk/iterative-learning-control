function [ws,Rqu] = CalWbyq4Sleap(StartQuat, EndQuat,theta,tu)
    % 通过四元数来计算角速度  这里正对四元数  这里的角速度是关于位置的，不是关于时间的  位置为0到1的参数
   at=sin((1-tu)*theta)/sin(theta);
   bt=sin(tu*theta)/sin(theta);
   Rqu=at*StartQuat+bt*EndQuat;
   %再计算下四元数的导数
   d4q=(theta/sin(theta))*(-cos((1-tu)*theta)*StartQuat+cos(tu*theta)*EndQuat);
   q4x=[Rqu(1),-Rqu(2),-Rqu(3),-Rqu(4)];
   kss = 2*fast_quatmultiply(d4q, q4x);
   ws =kss(2:4);
end