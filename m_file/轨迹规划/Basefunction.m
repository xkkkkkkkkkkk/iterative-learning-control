function Bik_u = Basefunction(i, k, u, Nodevector)

if k == 0 % 0次样条
    if u >= Nodevector(i+1) && u < Nodevector(i+2)
        Bik_u = 1;
    else
        Bik_u = 0;
    end
else
    Length1 = Nodevector(i+k+1) - Nodevector(i+1);
    Length2 = Nodevector(i+k+2) - Nodevector(i+2);   % 支撑区间的长度
    if Length1 == 0  % 规定0/0 = 0
        Length1 = 1;
    end
    if Length2 == 0
        Length2 = 1;
    end
    Bik_u = (u - Nodevector(i+1))/Length1 * Basefunction(i, k-1, u, Nodevector)...
        + (Nodevector(i+2) - u)/Length2 * Basefunction(i+1, k-1, u, Nodevector);
end