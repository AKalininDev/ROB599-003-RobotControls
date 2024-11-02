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
            inequalitySolution = (-inf < var) & (var < inf);
            inequalitiesSol = (inequalitiesSol) & (inequalitySolution);
            continue;
            
        elseif rhs_eval > 0 && isequal(inequalities(i), (rhs_s > 0))
            inequalitySolution = (-inf < var) & (var < inf);
            inequalitiesSol = (inequalitiesSol) & (inequalitySolution);
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
            right_bound = rhsProbes(j);
            
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

inequalitiesSol = simplify(expand((inequalitiesSol)));

end