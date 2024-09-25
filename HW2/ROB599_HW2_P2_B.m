%% Problem 2 B.
% Determining the Stability of PID controller

syms kp kd ki l

% State Space Definition 
A = [0, 1, 0; 24.525-kp*12.5, -4.167 - kd*12.5, 12.5*ki; -1, 0, 0]


% Compute the characteristic equation
char_eq = det(l*eye(3) - A);

% Simplify the result
char_eq_simplified = simplify(char_eq);

% Display the result
disp('Characteristic Equation:')
disp(char_eq_simplified)

%% Stability Criteria

% Routh-Hurwitz Matrix Definition

H = [ 12.5*kd + 4.167, 12.5*ki, 0;  1, 12.5*kp - 24.525, 0;  0, 12.5*kd + 4.167, 12.5*ki]


% Solve the inequalities for the polynomial coefficients
solve(12.5*kd + 4.167 > 0, kd, 'ReturnConditions', true).conditions
solve(12.5*kp - 24.525 > 0, kp, 'ReturnConditions', true).conditions
solve(12.5*ki> 0, kp, 'ReturnConditions', true).conditions


% Solving for principle minors
% PM 1:
PM1 = [1, 12.5*kp - 24.525; 12.5*kd + 4.167, 12.5*ki]
d1 = det(PM1) == 0
solve(d1, ki)
