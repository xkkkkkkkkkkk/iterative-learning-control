function RPY=RMatrix2RPY(rot)
RPY=zeros(3,1);
  sy = sqrt(rot(1,1)^2 + rot(2,1)^2);
  if(sy > 1e-6)
	 RPY(1) = atan2(rot(3,2), rot(3,3));
	 RPY(2) = atan2(-rot(3,1), sy);
	 RPY(3) = atan2(rot(2,1), rot(1,1));
  else
      if (rot(3,1) < 0)
          RPY(1) = atan2(rot(1,2), rot(2,2));
		  RPY(2) = pi / 2;
		  RPY(3) = 0;
      else
          RPY(1) = -atan2(rot(1,2), rot(2,2));
		  RPY(2) = -pi / 2;
		  RPY(3) = 0;
      end

  end

