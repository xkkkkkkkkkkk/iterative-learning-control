%{
Function: rpy2rotm
Description: rpy转旋转矩阵R， R= rotz(c) * roty(b) * rotx(a)
Input: rpy角度(单位rad)
Output: 旋转矩阵R
Author: Marc Pony(marc_pony@163.com)
%}
function R = rpy2rotm(rpy)

a = rpy(1);
b = rpy(2);
c = rpy(3);

sinA = sin(a);
cosA = cos(a);
sinB = sin(b);
cosB = cos(b);
sinC = sin(c);
cosC = cos(c);

R = [cosB*cosC,  cosC*sinA*sinB - cosA*sinC,  sinA*sinC + cosA*cosC*sinB
    cosB*sinC, cosA*cosC + sinA*sinB*sinC, cosA*sinB*sinC - cosC*sinA
    -sinB, cosB*sinA,  cosA*cosB];

end
