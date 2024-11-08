%% Function that Solves Real Strict Symbolic Inequalities
% It probes points between the real roots of the inequality and checks the signs.

function inequalitiesSol = solveRealStrictInequalities(inequalities, var)
% Solve inequalities by finding the roots of rhs = 0, and then checking the signs between the roots

inequalitiesSol = sym(true);

for i=1:length(inequalities)
    
    inequalitySolution = sym(false);
    rhs_s = rhs(inequalities(i));
    roots = solve(rhs_s == 0, var, 'Real', true);
    roots = double(real(roots));
    if isempty(roots)
        
        rhs_eval = eval(subs(rhs_s, var, 0));
        
        if rhs_eval < 0 && isequal(inequalities(i), (rhs_s < 0))
            continue;
            
        elseif rhs_eval > 0 && isequal(inequalities(i), (rhs_s > 0))
            continue;
        else
            inequalitiesSol = false;
            return;
        end
        
    end
    
    % Add probes to the left and to the right of the roots
    probeLocations = [min(roots) - 10, max(roots) + 10];
    
    % Add midpoints between the roots
    roots = sort(roots);
    midPoints = ((roots(1:end-1) + roots(2:end))/2)';
    
    % Complete and sort the probe locations
    probeLocations = sort([probeLocations, midPoints]);
    rhsProbes = eval(subs(rhs_s, var, probeLocations));
    
    
    for j=1:length(rhsProbes)
        
        isSolution = false;
        
        if j == 1
            left_bound = -inf; % left bound is -inf for the first probe
            right_bound = roots(j);
            
        elseif j == length(rhsProbes)
            left_bound = roots(j - 1);
            right_bound = inf; % right bound is inf for the last probe
            
        else
            left_bound = roots(j - 1);
            right_bound = roots(j);
        end
        
        if (rhsProbes(j) < 0 && isequal(inequalities(i), (rhs_s < 0)))
            isSolution = true;
        end
        
        if (rhsProbes(j) > 0 && isequal(inequalities(i), (rhs_s > 0)))
            isSolution = true;
        end
        
        if isSolution
            interval = (var > left_bound) & (var < right_bound);
            inequalitySolution = (inequalitySolution) | (interval);
        end
    end
    inequalitiesSol = (inequalitiesSol) & (inequalitySolution);
end

inequalitiesSol = simplify(expand(simplify(inequalitiesSol)));

end



%
%
%
% eq1 = Jm*theta_m_ddot + Bm*theta_m_dot - k*(theta_l - theta_m) == u
% eq2 = Jl*theta_l_ddot + Bl*theta_l_dot + k*(theta_l - theta_m) == 0
% eq3 = u == kp*(theta_cmd - theta_l) - kd*theta_l_dot

% H_s = theta_l == (k/(pl_s*pm_s)) * (k*theta_l + (kp + kd*s)*(r-theta_l));
% Q_s = theta_l == (k/(pl_s*pm_s)) * (k*theta_l + (10 + s)*r);
