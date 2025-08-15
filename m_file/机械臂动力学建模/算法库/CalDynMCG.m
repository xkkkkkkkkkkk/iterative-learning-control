function [M,C,GS]=CalDynMCG(DH,q,qd,qdd,m,I,CenP,G,Fext,n,mode)

  oqdd=[1,1,1,1,1,1];
  zqd=zeros(6,1);
  zqdd=zeros(6,1);
  Gn=[0,0,0]';
  B=[0.0,0.0,0.0,0.0,0.0,0.0];
  TF=[0,-0;
    0,-0.0;
    0.0,-0.0;
    0.0,-0.0;
    0.0,-0.0;
    0.0,-0.0;];
  Jm=[0.0000,0.0000,0.0000,0.0000,0.0000,0.0000];
  M=NewtonEulerDyn(DH,q,zqd,oqdd,m,I,CenP,Jm,B,TF,Gn,Fext,n,mode);

  GS=NewtonEulerDyn(DH,q,zqd,zqdd,m,I,CenP,Jm,B,TF,G,Fext,n,mode);

  C=NewtonEulerDyn(DH,q,qd,zqdd,m,I,CenP,Jm,B,TF,G,Fext,n,mode)-GS;

end