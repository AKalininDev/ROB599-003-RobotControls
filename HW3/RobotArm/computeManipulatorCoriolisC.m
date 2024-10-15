function C = computeManipulatorCoriolisC(x, param)
    
    % Interpretation of State Space
    q1dot = x(2); q2 = x(3); q2dot = x(4);
    
    % Import System Parameters
    m2 = param.m2;
    l1 = param.l1; lc2 = param.lc2; 
    
    % Compute the Coriolis Matrix
    C = zeros(2, 2);
    C(1, 1) = -m2*l1*lc2*sin(q2)*q2dot;
    C(1, 2) = -m2*l1*lc2*sin(q2)*(q1dot + q2dot);
    C(2, 1) = m2*l1*lc2*sin(q2)*q1dot;
    C(2, 2) = 0;

end

