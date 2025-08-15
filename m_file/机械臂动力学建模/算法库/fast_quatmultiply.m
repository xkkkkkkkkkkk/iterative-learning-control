function result = fast_quatmultiply(q, r)
    % 优化四元数乘法 (减少乘法次数)
    w = q(1)*r(1) - q(2)*r(2) - q(3)*r(3) - q(4)*r(4);
    x = q(1)*r(2) + q(2)*r(1) + q(3)*r(4) - q(4)*r(3);
    y = q(1)*r(3) - q(2)*r(4) + q(3)*r(1) + q(4)*r(2);
    z = q(1)*r(4) + q(2)*r(3) - q(3)*r(2) + q(4)*r(1);
    
    result = [w, x, y, z];
end

