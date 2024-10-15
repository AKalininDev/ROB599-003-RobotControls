function N = computeManipulatorGravityN(x, param)
    
    % Interpretation of State Space
    q1 = x(1); q2 = x(3);
    
    % Import System Parameters
    m1 = param.m1; m2 = param.m2; g = param.g;
    l1 = param.l1; lc1 = param.lc1; lc2 = param.lc2; 
    
    % Compute the Gravity Terms
    N = zeros(2, 1);
    N(1) = m1*g*lc1*cos(q1) + m2*g*(l1*cos(q1) + lc2*cos(q1 + q2));
    N(2) = m2*g*lc2*cos(q1 + q2);

end

