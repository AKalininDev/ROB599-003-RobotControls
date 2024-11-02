%% ROB599_HW4_PA
% Simiulating a PD Controller with Motor Angle Feedback for Series Elastic Actuator.
% Problem 1A: Derive the state space representation of the system.
% Problem 2A: Add numerical values of the parameters to the system and determine kd range such that the system is stable using Routh Hurwitz.
% Problem 3A: Determine the transfer function of the system.
% Problem 4A: Create root locus plot for the system.

%% Clean Up
clear
clc

%%
% <html><h2> Problem 1A </h2></html>
%
% Deriving the State Space Representation of the System.

%%
% <html><h3> Define System Variables </h3></html>

% Inertia Terms
syms Jm Jl

% Damping Terms
syms Bm Bl

% Elastic Term
syms k

% State Variables
syms theta_m theta_m_dot theta_m_ddot
syms theta_l theta_l_dot theta_l_ddot
syms u

% Control Terms
syms kp kd theta_cmd

%%
% <html><h3> Define EOMs </h3></html>
eq1 = Jm*theta_m_ddot + Bm*theta_m_dot - k*(theta_l - theta_m) == u
eq2 = Jl*theta_l_ddot + Bl*theta_l_dot + k*(theta_l - theta_m) == 0
eq3 = u == kp*(theta_cmd - theta_m) - kd*theta_m_dot

%%
% <html><h3> Acquire State Space Representaiton </h3></html>

% State Space Variables
syms x1 x2 x3 x4
syms x2_dot x4_dot

% Substitute State Space Variables into EOMs
eq1_state = subs(eq1, [theta_m, theta_m_dot, theta_l, theta_l_dot, theta_m_ddot], [x1, x2, x3, x4, x2_dot]);
eq2_state = subs(eq2, [theta_m, theta_m_dot, theta_l, theta_l_dot, theta_l_ddot], [x1, x2, x3, x4, x4_dot]);
u_state = subs(eq3, [theta_m, theta_m_dot, theta_l, theta_l_dot], [x1, x2, x3, x4]);
u_state = solve(u_state, u);

% Solve for x2_dot
eq1_state = subs(eq1_state, u, u_state);
x2_dot = solve(eq1_state, x2_dot);
x2_dot = simplify(x2_dot);
x2_dot = collect(x2_dot, [x1, x2, x3, x4, theta_cmd]);

% Solve for x4_dot
x4_dot = solve(eq2_state, x4_dot);
x4_dot = simplify(x4_dot);
x4_dot = collect(x4_dot, [x1, x2, x3, x4, theta_cmd]);

% Form state equations
x_dot = [x2;
    x2_dot;
    x4;
    x4_dot]

% Extract A matrix and B vector (x_dot = Ax + B*theta_cmd)
A = jacobian(x_dot, [x1; x2; x3; x4]);
A = simplify(A)

B = jacobian(x_dot, theta_cmd);
B = simplify(B)

%%
% <html><h2> Problem 2 A </h2></html>
%
% Determine the range of kd such that the system is stable using Routh Hurwitz.

% Adding numerical values of the parameters
param.Jm = 0.0097;
param.Jl = 0.0097;

param.Bm = 0.04169;
param.Bl = 0.04169;

param.k = 100;

param.kp = 10;

A_eval = subs(A, [Jm, Jl, Bm, Bl, k, kp], [param.Jm, param.Jl, param.Bm, param.Bl, param.k, param.kp]);
B_eval = subs(B, [Jm, Jl, Bm, Bl, k, kp], [param.Jm, param.Jl, param.Bm, param.Bl, param.k, param.kp]);

A_num = vpa(A_eval, 3)
B_num = vpa(B_eval, 3)

%%
% <html><h3> Acquire Characteristic Equations </h3></html>
% Characteristic Equation: det(A - lambda*I) = 0

syms lambda
char_eq = det(A_eval - lambda*eye(4));
char_eq = collect(char_eq, lambda);
char_eq_num = vpa(char_eq, 3)

%%
% <html><h3> Construct Routh-Hurwitz Matrix </h3></html>

routh_matrix = getRouthMatrix(char_eq, lambda);
routh_matrix_num = vpa(routh_matrix, 3)

%%
% <html><h3> Compute Principle Minors of Routh-Hurwitz Matrix </h3></html>

principalMinors = computePrincipalMinors(routh_matrix);
showFactoredForm(principalMinors, kd);

%%
% <html><h3> Solve for Stability Requirements </h3></html>

%inequalities = sym([]);
inequalities = [0 < kd];
for i=1:length(principalMinors)
    inequality = principalMinors(i) > 0;
    inequalities = [inequalities, inequality];
end

inequalitiesSol = solveRealStrictInequalities(inequalities, kd);
inequalities_sol = vpa(inequalitiesSol, 3)


%%
% <html><h2> Helper Functions </h2></html>
%

% Display the factored form of the expression
function showFactoredForm(expressions, var)
factoredForm = sym([]);
for i=1:length(expressions)
    factoredForm = [factoredForm, prod(factor(expressions(i), var, "FactorMode", "real"))];
end

for i=1:length(factoredForm)
    
    % Display the factored form of the expression
    factoredFormEquation = vpa(factoredForm(i), 3)
end

end