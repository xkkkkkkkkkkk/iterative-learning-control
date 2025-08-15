function Y = Ymatrix_clc(q,dq,ddq,DH,Pr,g)
q1=q(1); q2=q(2); q3=q(3);q4=q(4); q5=q(5); q6=q(6);
dq1=dq(1); dq2=dq(2); dq3=dq(3);dq4=dq(4); dq5=dq(5); dq6=dq(6);
ddq1=ddq(1); ddq2=ddq(2); ddq3=ddq(3);ddq4=ddq(4); ddq5=ddq(5); ddq6=ddq(6);
% global a3 a4 d2 d4 d5 d6 g;
% global Pc11 Pc22 Pc33 Pc44 Pc55 Pc66;


 
Pc11 = Pr{1}';Pc22 = Pr{2}'; Pc33 = Pr{3}';
Pc44 = Pr{4}'; Pc55 = Pr{5}';  
Pc66 = Pr{6}';

% ×ø±êÏµÐý×ªÖá
axis1=[0 0 1]';axis2=[0 0 1]';axis3=[0 0 1]';axis4=[0 0 1]';axis5=[0 0 1]';axis6=[0 0 1]';
% ×ø±êÏµ±ä»»¾ØÕó


T01 = MDH(q1,DH(1,2),DH(1,3),DH(1,4));
T12 = MDH(q2,DH(2,2),DH(2,3),DH(2,4));
T23 = MDH(q3,DH(3,2),DH(3,3),DH(3,4));
T34 = MDH(q4,DH(4,2),DH(4,3),DH(4,4));
T45 = MDH(q5,DH(5,2),DH(5,3),DH(5,4));
T56 = MDH(q6,DH(6,2),DH(6,3),DH(6,4));

R01=T01(1:3,1:3);R10=R01';P01=T01(1:3,4);
R12=T12(1:3,1:3);R21=R12';P12=T12(1:3,4);
R23=T23(1:3,1:3);R32=R23';P23=T23(1:3,4);
R34=T34(1:3,1:3);R43=R34';P34=T34(1:3,4);
R45=T45(1:3,1:3);R54=R45';P45=T45(1:3,4);
R56=T56(1:3,1:3);R65=R56';P56=T56(1:3,4);
%% ÍâÍÆ
w00=[0 0 0]';dw00=[0 0 0]';dv00=[0 0 g]';
[w11,dw11,dv11,dvc11]=motion_para_clc(R10,dq1,ddq1,w00,dw00,dv00,P01,Pc11,axis1);
[w22,dw22,dv22,dvc22]=motion_para_clc(R21,dq2,ddq2,w11,dw11,dv11,P12,Pc22,axis2);
[w33,dw33,dv33,dvc33]=motion_para_clc(R32,dq3,ddq3,w22,dw22,dv22,P23,Pc33,axis3);
[w44,dw44,dv44,dvc44]=motion_para_clc(R43,dq4,ddq4,w33,dw33,dv33,P34,Pc44,axis4);
[w55,dw55,dv55,dvc55]=motion_para_clc(R54,dq5,ddq5,w44,dw44,dv44,P45,Pc55,axis5);
[w66,dw66,dv66,dvc66]=motion_para_clc(R65,dq6,ddq6,w55,dw55,dv55,P56,Pc66,axis6);
% ¸¨Öú¾ØÕóH
H1=getHi(w11,dw11,dv11);
H2=getHi(w22,dw22,dv22);
H3=getHi(w33,dw33,dv33);
H4=getHi(w44,dw44,dv44);
H5=getHi(w55,dw55,dv55);
H6=getHi(w66,dw66,dv66);
% ¸¨Öú¾ØÕóA
A1=getAi(w11,dw11,dv11);
A2=getAi(w22,dw22,dv22);
A3=getAi(w33,dw33,dv33);
A4=getAi(w44,dw44,dv44);
A5=getAi(w55,dw55,dv55);
A6=getAi(w66,dw66,dv66);
% ¸¨Öú¾ØÕóYf¡¢Yn
Yf6=[zeros(3,50) H6]; Yn6=[zeros(3,50) A6]; 
[Yf5, Yn5]=get_Yf_Yn(5,R56,H5,A5,Yf6,Yn6,P56);
[Yf4, Yn4]=get_Yf_Yn(4,R45,H4,A4,Yf5,Yn5,P45);
[Yf3, Yn3]=get_Yf_Yn(3,R34,H3,A3,Yf4,Yn4,P34);
[Yf2, Yn2]=get_Yf_Yn(2,R23,H2,A2,Yf3,Yn3,P23);
[Yf1, Yn1]=get_Yf_Yn(1,R12,H1,A1,Yf2,Yn2,P12);
% ÏßÐÔ¾ØÕóY
Y(1,1:60)=axis1'*Yn1;
Y(2,1:60)=axis2'*Yn2;
Y(3,1:60)=axis3'*Yn3;
Y(4,1:60)=axis4'*Yn4;
Y(5,1:60)=axis5'*Yn5;
Y(6,1:60)=axis6'*Yn6;
end

function K=getK(vec3)
K=[vec3(1)  vec3(2)  vec3(3)    0        0        0;
     0      vec3(1)    0      vec3(2)  vec3(3)    0;
     0        0      vec3(1)    0      vec3(2)  vec3(3)];
end

% ²æ»ý¾ØÕó
function S=getS(vec3)
S=[   0     -vec3(3)  vec3(2);
   vec3(3)     0     -vec3(1);
   -vec3(2) vec3(1)     0    ];
end

function Hi=getHi(wi,dwi,dvi)
Hi(1:3,7:9)=getS(dwi)+getS(wi)*getS(wi);
Hi(1:3,10)=dvi;
Hi(1:3,1:6)=zeros(3,6);
end

function Ai=getAi(wi,dwi,dvi)
Ai(1:3,1:6)=getK(dwi)+getS(wi)*getK(wi);
Ai(1:3,7:9)=-getS(dvi);
Ai(1:3,10)=zeros(3,1);
end

function [Yf, Yn] = get_Yf_Yn(id, R, H, A, Yf_next, Yn_next, Po)
c1=(id-1)*10+1;c2=(id-1)*10+10;
Yf=zeros(3,60);Yf(:,c1:c2)=H;
Yf=Yf+R*Yf_next;
Yn=zeros(3,60);Yn(:,c1:c2)=A;
Yn=Yn+R*Yn_next+getS(Po)*R*Yf_next;
end

function [w, dw ,dv ,dvc] = motion_para_clc(R_inv,dq,ddq,w_pre,dw_pre,dv_pre,Po,Pc,axis)
    w = R_inv*w_pre+dq*axis;
    dw = R_inv*dw_pre+cross(R_inv*w_pre,dq*axis)+ddq*axis;
    dv = R_inv*(cross(dw_pre,Po)+cross(w_pre,cross(w_pre,Po))+dv_pre);
    dvc = cross(dw,Pc)+cross(w,cross(w,Pc))+dv;
end
