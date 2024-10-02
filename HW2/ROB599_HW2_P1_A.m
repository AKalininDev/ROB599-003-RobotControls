%% Problem 1 A.
% Converting to State-Space Form. Linearizing the Model. Checking
% Stability.

%% Define System Parameters

sysPar.SI.m = 0.2     %kg
sysPar.SI.J = 0.006   %kgm2
sysPar.SI.l = 0.3     %m

sysPar.SI.gamma = 0.1 %Nms/rad
sysPar.SI.g = 9.81    %m2/sec

%% Define Equation of Motion
% Variables Definition
syms theta theta_dot theta_dot_dot 

syms J m
syms l

syms gamma 

syms g

syms F

% Equation of Motion
motionEquation = (J + m*l^2)*theta_dot_dot + gamma*theta_dot - m*g*l*sin(theta) == l*cos(theta)*F

%% Get the State-Space Representation
% Define State-Space Variables
syms x1 x2
syms x1_dot x2_dot

syms u

% Define the matrices
syms X_dot
syms X
syms U

X_dot = [x1_dot; x2_dot];
X = [x1; x2];
U = [u];

% Extract theta_dot_dot from the motion equation
theta_dot_dot_expr = solve(motionEquation, theta_dot_dot);

X_dot = subs(X_dot, x1_dot, theta_dot);
X_dot = subs(X_dot, x2_dot, theta_dot_dot_expr);

% Now, replace the thetas with xs to get the state-space representation
X_dot = subs(X_dot, {theta, theta_dot, F}, {x1, x2, u})

%% Linearize the Model Around the Equilibrium
% Define the equillibrium location

% State Equillibrium
syms xeq_1 xeq_2

xeq_1 = 0;
xeq_2 = 0;
X_eq = [xeq_1; xeq_2];

% Input Equillibrium
syms ueq
ueq = 0;
U_eq = [ueq];

% Get the X Jacobian at equillibrium
Xjacobian = jacobian(X_dot, X);
Xjacobian = subs(Xjacobian, X, X_eq);
Xjacobian = subs(Xjacobian, U, U_eq)

% Get the U Jacobian at equillibrium
Ujacobian = jacobian(X_dot, U);
Ujacobian = subs(Ujacobian, X, X_eq);
Ujacobian = subs(Ujacobian, U, U_eq)

% Evaluate using system parameters
XJacEval = eval(subs(Xjacobian, {m, J, l, gamma, g}, {sysPar.SI.m, sysPar.SI.J, sysPar.SI.l, sysPar.SI.gamma, sysPar.SI.g}))
UJacEval = eval(subs(Ujacobian, {m, J, l, gamma, g}, {sysPar.SI.m, sysPar.SI.J, sysPar.SI.l, sysPar.SI.gamma, sysPar.SI.g}))

%% Computing the Eigenvectors and Eigenvalues of the System
[V, D] = eig(XJacEval);

D
V