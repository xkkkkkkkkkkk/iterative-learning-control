function w=logR(R)
 %计算矩阵的对数
 trR=R(1,1)+R(2,2)+R(3,3);
 if(trR==-1)
   w=[R(1,3),R(2,3),1+R(3,3)]/(sqrt(2*(1+R(3,3))));
   return;
 end
 theta=acos(0.5*(trR-1));
 WR=(R-R')/(2*sin(theta));
 w=[WR(3,2),WR(1,3),WR(2,1)];

end