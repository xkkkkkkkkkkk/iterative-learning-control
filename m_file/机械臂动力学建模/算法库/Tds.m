 function Td = Tds(thea,d,a,alpha)%%标准发进行DH建模的杆件变换
    Td=[cos(thea) -sin(thea)*cos(alpha) sin(thea)*sin(alpha) a*cos(thea);
        sin(thea) cos(thea)*cos(alpha) -cos(thea)*sin(alpha) a*sin(thea);
        0         sin(alpha)             cos(alpha)            d;
        0           0                         0                 1;
        ];
 end
