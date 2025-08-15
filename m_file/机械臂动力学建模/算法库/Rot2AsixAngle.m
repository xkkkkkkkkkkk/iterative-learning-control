function Rtheta=Rot2AsixAngle(R)
%这里将旋转矩阵转化为轴角表示  表示为  轴和角相乘  主要用来做姿态阻抗控制
theta=acos((R(1,1)+R(2,2)+R(3,3)-1)/2);
    if(theta<1e-6)
        Rtheta=[0,0,0];
    else
        r=(1/(2*sin(theta)))*[R(3,2)-R(2,3),R(1,3)-R(3,1),R(2,1)-R(1,2)];
        Rtheta=theta*r;
    end
end