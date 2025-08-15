function [MaxVel,MaxAcc,MaxDec,S1,V1,A1,flag]=PlanCircleCalAcc_Vel(rad,L,S_vel,S_acc,E_vel,Max_v,Max_a,Min_a,Max_j,n)
  Amax=max(Max_a,abs(Min_a));
  va=sqrt(rad*Amax);
  %vS=((rad)*Jmax)^(1/3);  加加速度限制部分后续在做详细研究
  Vmax=va;
  Amaxq=500000;
  Amaxw=500000;
  Aminq=-500000;
  Aminw=-500000;
  Vmaxm1=100000;
  Vmaxm2=100000;
  Vmaxm3=100000;
  Vmaxm4=100000;
  if(Max_v<Vmax)
     Vmax=Max_v;
  end
flag=1;
  [S1,V1,A1,TZ,Distance,Velocity,ACC,JZ]=ScurvePlan(0,S_vel,S_acc,L,E_vel,Vmax,-Vmax,Max_a,-abs(Min_a),Max_j,-Max_j,n);
  %根据规划结果先计算加速度  两个加速度需要计算
  if(ACC(7)>0||ACC(8)>0)
      Amax1=sqrt((Velocity(7)^2/rad)^2+ACC(7)^2);
      Amax2=sqrt((Velocity(8)^2/rad)^2+ACC(8)^2);
     [Amax,locala]=max([Amax1,Amax2]);
    if(Amax<=Max_a)
      Amaxq=Max_a;
      Vmaxm1=Vmax;
      flag=flag*1;
    else
            flag=flag*0;
      if (((Velocity(6+locala)^2/rad)^2)/Max_a^2)>0.25
          Amaxq=0.75*Max_a;
      else
          Amaxq=sqrt(Max_a^2-(Velocity(6+locala)^2/rad)^2);%%需要处理求解出来为非实数的情况
      end
      ANmax1=sqrt(Max_a^2-Amaxq^2);
      Vn1=sqrt(rad*ANmax1);
        t=Amaxq/Max_j;
        Vmaxm=Vn1+0.5*Amaxq^2/Max_j;
      if(Vmaxm>Vmax)
        Vmaxm1=Vmax;
      else
        Vmaxm1=Vmaxm;
      end
    end

  end
  if(ACC(11)>0||ACC(12)>0)
      Amax1=sqrt((Velocity(11)^2/rad)^2+ACC(11)^2);
      Amax2=sqrt((Velocity(12)^2/rad)^2+ACC(12)^2);
      [Amax,localb]=max([Amax1,Amax2]);
      if(Amax<=abs(Max_a))
            flag=flag*1;
        Amaxw=Max_a;
        Vmaxm2=Vmax;
      else
              flag=flag*0;
        if (((Velocity(10+localb)^2/rad)^2)/Max_a^2)>0.25
            Amaxw=0.75*Max_a;
        else
            Amaxw=sqrt(Max_a^2-(Velocity(10+localb)^2/rad)^2);%%需要处理求解出来为非实数的情况
        end
        ANmax1=sqrt(Max_a^2-Amaxw^2);
        Vn1=sqrt(rad*ANmax1);
        t=Amaxw/Max_j;
        Vmaxm=Vn1+0.5*Amaxq^2/Max_j;
        if(Vmaxm>Max_v)
          Vmaxm2=Max_v;
        else
          Vmaxm2=Vmaxm;
        end
      end

  end
   Amax=min(Amaxq,Amaxw);
   if(Amax>=400000)
      Amax=Max_a;
   end

 if(ACC(7)<0||ACC(8)<0)
      Amax1=sqrt((Velocity(7)^2/rad)^2+ACC(7)^2);
      Amax2=sqrt((Velocity(7)^2/rad)^2+ACC(7)^2);
     [Amin,localb]=max([Amax1,Amax2]);
     if(Amin<=abs(Min_a))
           flag=flag*1;
       Aminq=abs(Min_a);
       Vmaxm3=Vmax;
     else
         flag=flag*0;
       if (((Velocity(6+localb)^2/rad)^2)/Min_a^2)>0.25
            Aminq=abs(0.75*Min_a);
        else
            Aminq=sqrt(Min_a^2-(Velocity(6+localb)^2/rad)^2);%%需要处理求解出来为非实数的情况
        end
        ANmax1=sqrt(Min_a^2-Aminq^2);
        Vn1=sqrt(rad*Aminq1);
        t=Aminq/Max_j;
        Vmaxm=Vn1+0.5*Aminq^2/Max_j;
        if(Vmaxm>Max_v)
          Vmaxm3=Max_v;
        else
          Vmaxm3=Vmaxm;
        end
     end

  end

  if(ACC(11)<0||ACC(12)<0)
      Amax1=sqrt((Velocity(11)^2/rad)^2+ACC(11)^2);
      Amax2=sqrt((Velocity(12)^2/rad)^2+ACC(12)^2);
      [Amin,localb]=max([Amax1,Amax2]);
      if(Amin<=abs(Min_a))
        Aminw=-abs(Min_a);
        Vmaxm4=Vmax;
        flag=flag*1;
      else
        flag=flag*0;
        if (((Velocity(10+localb)^2/rad)^2)/Min_a^2)>0.25
            Aminw=abs(0.75*Min_a);
        else
            Aminw=sqrt(Min_a^2-(Velocity(10+localb)^2/rad)^2);%%需要处理求解出来为非实数的情况
        end
        ANmax1=sqrt(Min_a^2-Aminw^2);
        Vn1=sqrt(rad*ANmax1);
        t=Aminw/Max_j;
        Vmaxm=Vn1+0.5*Aminw^2/Max_j;
        if(Vmaxm>Max_v)
          Vmaxm4=Max_v;
        else
          Vmaxm4=Vmaxm;
        end
      end

  end
    Amin=max(Aminq,-Aminw);
   if(Amin<=-400000)
      Amin=Min_a;
   end
    MaxAcc=Amax;
    MaxDec=Amin;
MaxVel=min([Vmaxm1,Vmaxm2,Vmaxm3,Vmaxm4]);


end
