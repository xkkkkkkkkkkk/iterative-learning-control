clear all;
close all;
m=20;
n=100;
gama=0.9;
x1(1:m,1:n)=0;
x2(1:m,1:n)=0;
u(1:m,1:n)=0;
y(1:m,1:n)=0;
e(1:m,1:n)=0;
err(1:m,1:n)=0;

% b=beronulli(1,1000,1);
% w=randn(m,n)/5;
for n=1:100  
    y_d(n) = sin(8*(n-1)/50); 
%y_d(n) = 5*sin(n*pi/100)+0.3*cos(n*pi/100);   
%     if n<=300
%         y_d(n) =5*(-1).^round(n/100); 
%     else if n<=700  
%         y_d(n) = 5*sin(n*pi/100)+0.3*cos(n*pi/100);    
%     else    
%         y_d(n) =5*(-1).^round(n/100);
%     end
%     end
            
   %y_d(n)=12*n.^2*(n-1);
   %y_d(n) = sin(2*pi*n/50);
end
for k=1:m
    
    for t=2:n-1   
      % 描述系统状态
      x1(k,t)=(-0.8*x1(k,t-1))-0.22*x2(k,t-1)+0.5*u(k,t-1);
      x2(k,t)=x1(k,t-1)+u(k,t-1);
      y(k,t-1)=(x1(k,t-1)+0.5*x2(k,t-1));
      e(k,t-1)=y_d(t-1)-y(k,t-1);
      err(k,t-1)=abs(e(k,t-1)).^2;      
    end
    for t=2:n-1  
        u(k+1,t-1)=u(k,t-1)+gama*e(k,t)+0.3*e(k,t-1);    
    end
    errn(k)=max(err(k,:));
end
 plot(y_d(1:90),'-r');hold on
 plot(y(1,1:90));hold on;
 plot(y(2,1:90));hold on;
 plot(y(3,1:90));hold on;
 plot(y(5,1:90));hold on;
 plot(y(8,1:90));hold on;
 plot(y(10,1:90));hold on;
 plot(y(15,1:90));hold on;
figure 
% plot(err(5,1:90));    %误差迭代变化图
plot(err(12,1:20));hold on
plot(err(15,1:20));hold on
k=1:20;
figure;
plot(k,errn(k)) %最大误差迭代收敛图
