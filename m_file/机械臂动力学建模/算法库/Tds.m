 function Td = Tds(thea,d,a,alpha)%%��׼������DH��ģ�ĸ˼��任
    Td=[cos(thea) -sin(thea)*cos(alpha) sin(thea)*sin(alpha) a*cos(thea);
        sin(thea) cos(thea)*cos(alpha) -cos(thea)*sin(alpha) a*sin(thea);
        0         sin(alpha)             cos(alpha)            d;
        0           0                         0                 1;
        ];
 end
