function Tor=NewtonEulerDyns(DH,q,qd,qdd,m,I,CenP,Jm,B,TF,G,Fext,n,mode)
%ţ��ŷ��������ѧ��ģ  ����Ĭ�ϼ��ٱ�Ϊ1  ���ñ�׼DH����������ģ��
% ����ĸ�������Ϊ DH: DH���� ��׼����ģ��  ��λ m
% q qd qdd �ؽڵ�λ�� �ٶ� ���ٶ�  �����ƶ�
% m ������������   kg  (�������������������һ��)
% I ����������������ĵ�ת������
% CenP���������������������ϵ��λ��  (��������ϵ�ڿ���Ĺؽڴ�)
% Jm �����ת������
% B �����ճ��ϵ��
% TF �ؽ�Ħ����
% G ��������ʹ�С
% F ���� ��������
% n �������ɶȻ�е��
%mode ģʽ0 ��׼DH����ģʽ   1�Ľ�DH����ģʽ
TR=cell(n,1);
zt=cell(n,1);
R=cell(n+1,1);
r=cell(n+1,1);
v=cell(n+1,1);
w=cell(n+1,1);
dw=cell(n+1,1);
dv=cell(n+1,1);
dotvc=cell(n+1,1);
F=cell(n,1);
N=cell(n,1);
f=cell(n+1,1);
nt=cell(n+1,1);
Tor=[0,0,0,0,0,0];
%% �������  �����ٶ�
if(mode==0)
        for i=1:n
        TR{i}=Tds(q(i)+DH(i,5),DH(i,2),DH(i,3),DH(i,4));
        if(DH(i,4)==pi/2)
            zt{i}=[0,1,0]';
        elseif(DH(i,4)==-pi/2)
            zt{i}=[0,-1,0]';  
        else
            zt{i}=[0,0,1]';
        end
    end

    T{1}=[1,0,0,0;
          0,1,0,0;
          0,0,1,0;
          0,0,0,1];
    for i=2:n+1
        T{i}=T{i-1}*TR{i-1};
    end
    for i=1:n
        R{i}=TR{i}(1:3,1:3)';
    end
    for i=1:n
        R0{i}=T{i+1}(1:3,1:3)';
    end
    z=[0,0,1]';
    v{1}=[0,0,0]';
    w{1}=[0,0,0]';
    dv{1}=G;
    dw{1}=[0,0,0]';

    for i=1:n
       r{i}=R0{i}*(T{i+1}(1:3,4)-T{i}(1:3,4));
    end

    for i=1:n
       v{i+1}= R{i}*v{i}+cross(w{i},r{i}); 
       w{i+1}= R{i}*w{i}+R{i}*qd(i)*z;
       dw{i+1}=R{i}*(dw{i}+cross(w{i},qd(i)*z)+qdd(i)*z);
       dv{i+1}=R{i}*dv{i}+cross(dw{i+1},r{i})+cross(w{i+1},cross(w{i+1},r{i}));
       dotvc{i+1}=cross(dw{i+1},CenP{i})+cross(w{i+1},cross(w{i+1},CenP{i}))+dv{i+1};
    end

    for i=1:n
    F{i}=m(i)*dotvc{i+1};
    N{i}=I{i}*dw{i+1}+cross(w{i+1},I{i}*w{i+1});
    end

    f{n+1}=Fext(1:3)';
    nt{n+1}=Fext(4:6)';  %ĩ���ܵ���������Ť��

    R{n+1}=[1,0,0;0,1,0;0,0,1];

    for i=n:-1:1
      f{i}=R{i+1}'*f{i+1}+F{i};
      nt{i}=R{i+1}'*nt{i+1}+cross(CenP{i},F{i})+cross(r{i},f{i})+N{i};
    end


    d=0;
    for i=1:n
       if(qd(i)>0)
           k=1;
           d=TF(i,k);
       elseif(qd(i)<0)
           k=2;
           d=TF(i,k);
       else
           d=0;
       end
       Tor(i)=dot(nt{i},zt{i})+Jm(i)*qdd(i)+B(i)*qd(i)+d;
    end
else
    %%MDH
      for i=1:n
        TR{i}=MDH(q(i)+DH(i,5),DH(i,2),DH(i,3),DH(i,4));
%         if(DH(i,4)==pi/2)
%             zt{i}=[0,1,0]';
%         elseif(DH(i,4)==-pi/2)
%             zt{i}=[0,-1,0]';  
%         else
%             zt{i}=[0,0,1]';
%         end
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
    
    for i=1:n
       w{i+1}= R{i}*w{i}+qd(i)*z;
       dw{i+1}=R{i}*dw{i}+cross(R{i}*w{i},qd(i)*z)+qdd(i)*z;
       dv{i+1}=R{i}*(dv{i}+cross(dw{i},r{i})+cross(w{i},cross(w{i},r{i})));
       dotvc{i+1}=cross(dw{i+1},CenP{i})+cross(w{i+1},cross(w{i+1},CenP{i}))+dv{i+1};
    end
    
    for i=1:n
        F{i}=m(i)*dotvc{i+1};
        N{i}=I{i}*dw{i+1}+cross(w{i+1},I{i}*w{i+1});%% 
    end
    
        f{n+1}=Fext(1:3)';
    nt{n+1}=Fext(4:6)';  %ĩ���ܵ���������Ť��

    R{n+1}=[1,0,0;0,1,0;0,0,1];

    for i=n:-1:1
      f{i}=R{i+1}'*f{i+1}+F{i};
      nt{i}=R{i+1}'*nt{i+1}+cross(CenP{i},F{i})+cross(r{i+1},(R{i+1}'*f{i+1}))+N{i};
    end
    
     d=0;
    for i=1:n
       if(qd(i)>0)
           k=1;
           d=TF(i,k);
       elseif(qd(i)<0)
           k=2;
           d=TF(i,k);
       else
           d=0;
       end
       Tor(i)=nt{i}(3)+Jm(i)*qdd(i)+B(i)*qd(i)+d;
    end  
end
end