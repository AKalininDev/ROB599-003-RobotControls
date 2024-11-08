%% ROB599_HW4_PA
% Simiulating a PD Controller with Motor Angle Feedback for Series Elastic Actuator.

%% Clean Up
clear
clc
%% Problem 1A
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
eq1 = Jm*theta_m_ddot + Bm*theta_m_dot - k*(theta_l - theta_m) == u;
eq2 = Jl*theta_l_ddot + Bl*theta_l_dot + k*(theta_l - theta_m) == 0;
eq3 = u == kp*(theta_cmd - theta_m) - kd*theta_m_dot;

% Display Equations
pretty(eq1)
pretty(eq2)
pretty(eq3)
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
    x4_dot];
disp("X_dot = "); pretty(x_dot)

% Extract A matrix and B vector (x_dot = Ax + B*theta_cmd)
A = jacobian(x_dot, [x1; x2; x3; x4]);
A = simplify(A);
disp("A = "); pretty(A)

B = jacobian(x_dot, theta_cmd);
B = simplify(B)
%% Problem 2A
% Determine the range of kd such that the system is stable using Routh Hurwitz.
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
%
% Characteristic Equation: det(A - lambda*I) = 0
syms lambda
char_poly = det(A_eval - lambda*eye(4));
char_poly = collect(char_poly, lambda);
disp('Characteristic Polynomial: ');
pretty(vpa(char_poly, 3))
%%
% <html><h3> Construct Routh-Hurwitz Matrix </h3></html>
H = getRouthMatrix(char_poly, lambda);
disp('Routh-Hurwitz Matrix: ');
pretty(vpa(H, 3))

%%
% <html><h3> Compute Principle Minors of Routh-Hurwitz Matrix </h3></html>
principalMinors = computePrincipalMinors(H);
% showFactoredForm(principalMinors, kd);

%%
% <html><h3> Solve for Stability Requirements </h3></html>
inequalities = [0 < kd];
for i=1:length(principalMinors)
    inequality = principalMinors(i) > 0;
    inequalities = [inequalities, inequality];
end

inequalitiesSol = solveRealStrictInequalities(inequalities, kd);
disp("Stability Requirements: ");
pretty(vpa(inequalitiesSol, 3))
%% Problem 3A
% Acquire the transfer function of the system.

% Define New Variables
syms s
syms pl_s pm_s r

% Solve for Transfer Function theta_l/r
H_s = theta_l == (k/(pl_s*pm_s)) * (k*theta_l + (kp + kd*s)*(r-pl_s*theta_l/k));
H_s = solve(H_s, theta_l)/r;
H_s = simplify(subs(H_s, [pl_s, pm_s], [Jl*s^2+Bl*s+k, Jm*s^2+Bm*s+k]));
H_s = collect(H_s, s);
disp("Closed Loop Transfer Function");
disp('H_s = '); disp(H_s)

H_s_eval = vpa(subs(H_s, [Jm, Jl, Bm, Bl, k], [param.Jm, param.Jl, param.Bm, param.Bl, param.k]), 3);
disp('H_s_eval = ');disp(H_s_eval)

H_s_eval = vpa(subs(H_s, [Jm, Jl, Bm, Bl, k, kp], [param.Jm, param.Jl, param.Bm, param.Bl, param.k, param.kp]), 3);
disp('H_s_eval = '); disp(H_s_eval)
%% Problem 4A
% Get the Root Locus Plot of the system.

%%
% <html><h3> Acquire Open Loop Transfer Function L(s). </h3></html>

% Solve for the Forward Transfer Function
Q_s = theta_l == (k/(pl_s*pm_s)) * (k*theta_l + (10 + s)*r);
Q_s = solve(Q_s, theta_l)/r;
Q_s = subs(Q_s, [pl_s, pm_s], [Jl*s^2+Bl*s+k, Jm*s^2+Bm*s+k]);

% Solve for the Feedback Transfer Function
P_s = pl_s/k;
P_s = subs(P_s, pl_s, Jl*s^2+Bl*s+k);

% Get the Open Loop Transfer Function
SYS = subs(Q_s * P_s, [Jm, Jl, Bm, Bl, k], [param.Jm, param.Jl, param.Bm, param.Bl, param.k]);
SYS = collect(SYS, s);
[num, den] = numden(SYS);
num = sym2poly(num);
den = sym2poly(den);
L_s = tf(num, den);

disp('Open Loop Transfer Function');
disp('L(s) = '); pretty(SYS)
%%
% <html><h3> Plot the Root Locus Using Open Loop Transfer Function L(s). </h3></html>
figure('Color', 'white', 'Position', [100, 100, 1200, 800]);
rlocus(L_s)
set(findall(gca, 'Type', 'Line'),'LineWidth', 3);
sgrid();
title('Figure 1A. Series Elastic Actuator. Motor Angle Feedback. Root Locus, Kd Range. Kp/Kd = 10.', 'FontWeight', 'bold', 'FontSize', 24);
xlabel('Real Axis', 'FontWeight', 'bold', 'FontSize', 18);
ylabel('Imaginary Axis', 'FontWeight', 'bold', 'FontSize', 18);
grid on;
box on;
set(gca, 'FontSize', 14);

% Save the figure
print(gcf, 'Figure 1A. Root Locus of Motor Angle Feedback.png', '-dpng', '-r300');  % Save as PNG with 300 DPI resolution
%% Problem 5A
% Initialize Simulink Parameters for the System.
% Adding numerical values of the parameters

%%
% <html><h3> Redefining the Numerical Parameters </h3></html>

param.Jm = 0.0097;
param.Jl = 0.0097;

param.Bm = 0.04169;
param.Bl = 0.04169;

param.k = 100;

param.kp = 10;
param.kd = 0.05;
%%
% <html><h3> Run the Simulink Model </h3></html>
simOut = sim('ROB599_HW4_P5', 'SimulationMode', 'normal', 'StopTime', '10');
%%
% <html><h3> Pull The Simulation Results for Plotting. </h3></html>

% Get the Step Response Data (Motor Angle Feedback)
motorStepResponseTime = simOut.MotorStepResponse.Time;
motorStepResponse = simOut.MotorStepResponse.Data;

% Get the Step Response (Load Angle Feedback)
loadStepResponseTime = simOut.LoadStepResponse.Time;
loadStepResponse = simOut.LoadStepResponse.Data;

% Get the Step Response (Reference Input)
signalStepResponseTime = simOut.StepInputSignal.Time;
signalStepResponse = simOut.StepInputSignal.Data;
%%
% <html><h3> Create Figure for Motor Angle Feedback Step Response. </h3></html>

figure('Color', 'white', 'Position', [100, 100, 1200, 800]);
plot(motorStepResponseTime, motorStepResponse, 'LineWidth', 3);
hold on;
set(gca, 'FontSize', 14);
plot(signalStepResponseTime, signalStepResponse, '--', 'LineWidth', 3);
title('Figure 2.1A. Motor Angle Feedback Controller Response. Kp = 10, Kd = 0.05', 'FontWeight', 'bold', 'FontSize', 24);
xlabel('Time (s)', 'FontWeight', 'bold', 'FontSize', 18);
ylabel('Angle (rad)', 'FontWeight', 'bold', 'FontSize', 18);
legend('Load Angle', 'Reference Input');
grid on;
box on;

% Save motor angle feedback plot
print(gcf, 'Figure 2.1A. Motor Angle Feedback Controller Response.png', '-dpng', '-r300');
%%
% <html><h3> Create Figure for Load Angle Feedback Step Response. </h3></html>

figure('Color', 'white', 'Position', [100, 100, 1200, 800]);
plot(loadStepResponseTime, loadStepResponse, 'LineWidth', 3);
hold on;
set(gca, 'FontSize', 14);
plot(signalStepResponseTime, signalStepResponse, '--', 'LineWidth', 3);
title('Figure 2.2A. Load Angle Feedback Controller Response. Kp = 10, Kd = 0.05', 'FontWeight', 'bold', 'FontSize', 24);
xlabel('Time (s)', 'FontWeight', 'bold', 'FontSize', 18);
ylabel('Angle (rad)', 'FontWeight', 'bold', 'FontSize', 18);
legend('Load Angle', 'Reference Input');
grid on;
box on;

% Save load angle feedback plot
print(gcf, 'Figure 2.2A. Load Angle Feedback Controller Response.png', '-dpng', '-r300');
%% Helper/Debugging Functions

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