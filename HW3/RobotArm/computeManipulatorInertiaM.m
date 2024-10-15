function M = computeManipulatorInertiaM(x, param)
    
    % Interpretation of State Space
    q2 = x(3);
    
    % Import System Parameters
    m1 = param.m1; m2 = param.m2; I1 = param.I1; I2 = param.I2;
    l1 = param.l1; lc1 = param.lc1; lc2 = param.lc2; 
    
    % Compute the Ineertia Matrix
    M = zeros(2, 2);
    M(1,1) = m1*lc1^2 + m2*(l1^2 + lc2^2 + 2*l1*lc2*cos(q2))+I1+I2; 
    M(1,2) = m2*(lc2^2+l1*lc2*cos(q2))+I2;
    M(2,1) = M(1,2);
    M(2,2) = m2*lc2^2+I2;

end

