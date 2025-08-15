function Y=Dynamic_Linear(DH,q,qd,qdd,G,n)
%动力学方程线性化 基于牛顿欧拉法
    for i=1:n
        TR{i}=MDH(q(i)+DH(i,5),DH(i,2),DH(i,3),DH(i,4));
    end
    for i=1:n
        R{i}=TR{i}(1:3,1:3)';
        r{i}=TR{i}(1:3,4);
    end
    r{n+1}=[0,0,0]';
    z=[0,0,1]';
    v{1}=[0,0,0]';
    w{1}=[0,0,0]';
    dv{1}=G;
    dw{1}=[0,0,0]';
    
    %先计算末端线加速度 角速度  角加速度
    for i=1:n
       w{i+1}= R{i}*w{i}+qd(i)*z;
       dw{i+1}=R{i}*dw{i}+cross(R{i}*w{i},qd(i)*z)+qdd(i)*z;
       dv{i+1}=R{i}*(dv{i}+cross(dw{i},r{i})+cross(w{i},cross(w{i},r{i})));
    end
    
    % 后续来计算矩阵Y中的U  U矩阵包括A和Q
    
    for i=1:n
        A{i}=zeros(n,10);%先将A矩阵初始化为0
    end
     for i=1:n
        A{i}(1:3,1)=dv{i+1};%
        A{i}(1:3,2:4)=S_alg(dw{i+1})+S_alg(w{i+1})*S_alg(w{i+1});
        A{i}(4:6,2:4)= -S_alg(dv{i+1});
        A{i}(4:6,5:10) = K_alg(dw{i+1})+S_alg(w{i+1})*K_alg(w{i+1});%  这个参数可能存在问题  需要详细分析
     end
    
     for i=2:n  %QT  
         QT{i-1}=[R{i}' zeros(3,3);S_alg(r{i})*R{i}' R{i}'];
     end
    
     for i=1:n  %QT  
         for  j=i:n  %QT  
            if(i==j)
                Q{i,j}=A{i};
            else
                Q{i,j}=TranMalt(QT,i,j)*A{j};
         end
         end
     end
     
     if(n==2)
         YT=[Q{1,1},Q{1,2};
          zeros(6,10), Q{2,2}];
      Y=[YT(6,:);YT(12,:)];
     elseif(n==3)
           YT=[Q{1,1},Q{1,2},Q{1,3};
          zeros(6,10), Q{2,2},Q{2,3};
          zeros(6,10),zeros(6,10),Q{3,3}];
            Y=[YT(6,:);YT(12,:);YT(18,:)];
     elseif(n==6)
             YT = [Q{1,1} Q{1,2} Q{1,3} Q{1,4} Q{1,5} Q{1,6};
             zeros(6,10) Q{2,2} Q{2,3} Q{2,4} Q{2,5} Q{2,6};
             zeros(6,10) zeros(6,10) Q{3,3} Q{3,4} Q{3,5} Q{3,6};
             zeros(6,10) zeros(6,10) zeros(6,10) Q{4,4} Q{4,5} Q{4,6};
             zeros(6,10) zeros(6,10) zeros(6,10) zeros(6,10) Q{5,5} Q{5,6};
             zeros(6,10) zeros(6,10) zeros(6,10) zeros(6,10) zeros(6,10) Q{6,6};];
             Y=[YT(6,:);YT(12,:);YT(18,:);YT(24,:);YT(30,:);YT(36,:)];
     end
    

end