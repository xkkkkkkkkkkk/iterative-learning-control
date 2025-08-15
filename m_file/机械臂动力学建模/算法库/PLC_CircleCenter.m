function [Bcenter,rad,TB,theta] = PLC_CircleCenter(p1, p2, p3,mode,IFclockwise)
%%mode=1 经过三点的圆弧  mode=2 给定圆心和两点的圆弧  mode=3 给定法向量的方式
%% 计算在起始点p1为原点 p1p3为X轴的坐标系下的 圆心 半径 新坐标系下的Mps 对应Mp1  和新坐标系C相对于基坐标系B的关系T
%这里的TB为以圆心为原点 起点为x轴方向的坐标系和基坐标系的关系
if size(p1,2)~=3 || size(p2,2)~=3 || size(p3,2)~=3
    fprintf('输入点维度不一致\n');rad = -1;return;
end
n = size(p1,1);
if size(p2,1)~=n || size(p3,1)~=n
    fprintf('输入点维度不一致\n');rad = -1;return;
end
rad=1;
  v1 = p3 - p1;
  v2 = p2 - p1;
if(mode~=3)
    % 计算p1到p2的单位向量和p1到p3的单位向量
  % 检查点是否相同

  if find(norm(v1)==0) | find(norm(v2)==0) %%ok<OR2>
      fprintf('输入点不能一样\n');rad = -1;return;
  end
  v1n = v1/norm(v1);
  v2n = v2/norm(v2);
  % 计算圆平面上的单位法向量
  % 检查三点是否共线
  nv = cross(v1n,v2n);
   if all(nv==0)
     rad = -2;
   end
  if find(sum(abs(nv),2)<1e-5)
      rad = -1;
  end
end
if(mode==1)
   if(rad<0)
    fprintf('三个点共线\n'); return;
   else
    % 计算新坐标系UVW轴
    u = v1n;
    w = cross(v1,v2)/norm(cross(v1,v2));
    v = cross(u,w);
    T=[u',v',w',p1';0 0 0 1];
    % 计算投影
    bx = dot(v1,u);
    cx = dot(v2,u);
    cy = dot(v2,v);

    % 计算圆心 在新的坐标系下
    h = ((cx - bx/2)^2 + cy^2 -(bx/2)^2)/(2*cy);
%%    if(cy<0)
%%    h=abs(h);
%%    else
%%    h=-abs(h);
%%    end
    center=[bx/2,h];
    %计算半径
    rad=norm(center);
    pc1=[0,0];
    pc3=[bx,0];
    vc1=(pc1-center)/norm((pc1-center));
    vc2=(pc3-center)/norm((pc3-center));

    %%在平面坐标系下再进行坐标变换 以圆心为平面坐标系下的圆点 计算新的轴
    ox=[vc1,0];
    oz =[0,0,1];
    oy = cross(ox,oz);
    %计算在新坐标系下的点的位置 即投影
    %即
    To= [ox' oy' oz' [center,0]';0 0 0 1];%%新的坐标系相对于老坐标系的关系
    ocenter=[0,0,0];
    opc1=inv(To)*[pc1,0,1]';%%变换到新坐标系下  在新坐标系下进行插补
    opc3=inv(To)*[pc3,0,1]';

    if opc3(2)<0
        theta13 = atan2(opc3(2),opc3(1)) + 2*pi;
    else
        theta13 = atan2(opc3(2),opc3(1));
    end
%%    if(IFclockwise)
%%      theta13=theta13-2*pi;
%%    else
%%      theta13=theta13;
%%    end
    TB=T*To;
    ss=TB*[rad*cos(theta13/2), rad*sin(theta13/2),0,1]';
    point=ss(1:3);
    p1po=point'-p1;
    pop3=p3-point';
    csts=cross(p1po,pop3);
    %  0顺时针  1逆时针
    if(csts(3)>0)||((csts(3)==0)&&(csts(2)>0))%%逆时针情况
       if(IFclockwise==1)
         theta=theta13;
       else
         theta=theta13-2*pi;
       end
    else
       if(IFclockwise==1)
         theta=theta13-2*pi;
       else
         theta=theta13;
       end
    end
    Bcenter=T*To*[0,0,0,1]';
   end
   elseif(mode==2)%%圆心 两点圆弧   可能圆心不在两点的中垂线上 需要计算平均值并重新调整辅助点的位置
     p12=(p2-p1);
     p13=(p3-p1);
 if(rad<0)
   if(abs(p12(1))<=0.0001&&abs(p13(1))<=0.0001)&&(abs(p12(2))>0.0001||abs(p13(2))>0.0001)&&(abs(p12(3))>0.0001||abs(p13(3))>0.0001)
  %%可以假设圆在YZ平面上
  w=[1,0,0];
  elseif(abs(p12(2))<=0.0001&&abs(p13(2))<=0.0001)&&(abs(p12(1))>0.0001||abs(p13(1))>0.0001)&&(abs(p12(3))>0.0001||abs(p13(3))>0.0001)
  %%可以假设圆在XZ平面上
  w=[0,1,0];
  elseif(abs(p12(3))<=0.0001&&abs(p13(3))<=0.0001)&&(abs(p12(1))>0.0001||abs(p13(1))>0.0001)&&(abs(p12(2))>0.0001||abs(p13(2))>0.0001)
  %%可以假设圆在XY平面上
  w=[0,0,1];
  else
  return;
  end
 end

   Bcenter=p2;
   rad1=norm(p2-p1);
   rad2=norm(p3-p2);

%%还是需要构建坐标系  计算相关点的位置
  if(abs(rad1-rad2)<=0.0001)
  Bcenter=p2;
  rad=rad1;
   %% 构建新的坐标系
     v1 = p1 - Bcenter;
     v2 = p3 - Bcenter;
     v1n = v1/norm(v1);
     v2n = v2/norm(v2);
     u = v1n;
     if(rad>0)
     w = cross(v2,v1)/norm(cross(v2,v1));
     end
     v = cross(w,u);
     TB=[u',v',w',Bcenter';0 0 0 1];
     P1x = dot(v1,u);
     P3x = dot(v2,u);
     P3y = dot(v2,v);
     if P3y<0
        theta = atan2(P3y,P3x) + 2*pi;
    else
        theta= atan2(P3y,P3x);
    end
    if(IFclockwise)
      theta=theta-2*pi;
    else
      theta=theta;
    end
  else %重新找圆心
      rad=(rad1+rad2)/2;
      v1 = p3 - p1;
      v2 = p2 - p1;
          % 计算新坐标系UVW轴
     u = v1n;
     v1n = v1/norm(v1);
     v2n = v2/norm(v2);
     if(rad>0)
     w = cross(v2,v1)/norm(cross(v2,v1));
     end
     v = cross(w,u);
     T=[u',v',w',p1';0 0 0 1]
    % 计算投影
     bx = dot(v1,u);
     cx = dot(v2,u);
     cy = dot(v2,v);
     %计算在P1坐标系下圆心的位置
     h=sqrt(rad^2-(bx/2)^2);
     if(cy<0)
       h=-h;
     end
     center=[bx/2,h];
     pc1=[0,0];
    pc3=[bx,0];
    vc1=(pc1-center)/norm((pc1-center));
    vc2=(pc3-center)/norm((pc3-center));

    %%在平面坐标系下再进行坐标变换 以圆心为平面坐标系下的圆点 计算新的轴
    ox=[vc1,0];
    oz =[0,0,1];
    oy = cross(ox,oz);
    %计算在新坐标系下的点的位置 即投影
    %即
    To= [ox' oy' oz' [center,0]';0 0 0 1];%%新的坐标系相对于老坐标系的关系
    ocenter=[0,0,0];
    opc1=inv(To)*[pc1,0,1]';%%变换到新坐标系下  在新坐标系下进行插补
    opc3=inv(To)*[pc3,0,1]';

    if opc3(2)<0
        theta13 = atan2(opc3(2),opc3(1)) + 2*pi;
    else
        theta13 = atan2(opc3(2),opc3(1));
    end
    if(IFclockwise)
      theta13=theta13-2*pi;
    else
      theta13=theta13;
    end
    theta=theta13;
    TB=T*To;
    Bcenter=T*To*[0,0,0,1]';
  end

else%% 给定向量来进行插补处理
  %有向量就可以构建圆的坐标系  加上半径参数  和上面计算方式相同
   %% 计算半径
   rad=norm(p2);
   disp1p3=norm(p1-p3);
   if(rad<(disp1p3/2))
     rad=disp1p3/2;
   end
   % 建立坐标系
   w=p2/norm(p2);
   u=(p3-p1)/norm(p3-p1);
   v = cross(u,w);
   T=[u',v',w',p1';0 0 0 1]
    % 计算投影
   bx = dot(v1,u);
   h=sqrt(rad^2-(bx/2)^2);
   if((p2(3)>0)||((p2(3)==0)&&(p2(2)>0)))
     flagss=1;%选择短圆弧
   else
     flagss=-1;%选择长圆弧
   end
   if(IFclockwise==1)&&(flagss==1)
      h=h;
   elseif(IFclockwise==0)&&(flagss==1)
      h=-h;
   elseif(IFclockwise==1)&&(flagss==-1)
     h=-h;
   else
     h=h;
   end
   center=[bx/2,h];
     pc1=[0,0];
    pc3=[bx,0];
    vc1=(pc1-center)/norm((pc1-center));
    vc2=(pc3-center)/norm((pc3-center));

    %%在平面坐标系下再进行坐标变换 以圆心为平面坐标系下的圆点 计算新的轴
    ox=[vc1,0];
    oz =[0,0,1];
    oy = cross(ox,oz);
    %计算在新坐标系下的点的位置 即投影
    %即
    To= [ox' oy' oz' [center,0]';0 0 0 1];%%新的坐标系相对于老坐标系的关系
    ocenter=[0,0,0];
    opc1=inv(To)*[pc1,0,1]';%%变换到新坐标系下  在新坐标系下进行插补
    opc3=inv(To)*[pc3,0,1]';

    if opc3(2)<0
        theta13 = atan2(opc3(2),opc3(1)) + 2*pi;
    else
        theta13 = atan2(opc3(2),opc3(1));
    end
    if(flagss==1)
       if(abs(theta13)>pi)
         theta13=theta13-2*pi;
       else
         theta13=theta13;
       end
  else
      if(abs(theta13)>pi)
         theta13=theta13;
       else
         theta13=theta13-2*pi;
      end
    end
    theta=theta13;
    TB=T*To;
    Bcenter=T*To*[0,0,0,1]';

end

%%上述相关计算方式可以进一步封装 结果几乎相同





end
