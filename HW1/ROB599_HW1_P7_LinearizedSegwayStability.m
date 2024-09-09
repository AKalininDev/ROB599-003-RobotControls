%% ROB599-HW1-Problem7
% Evaluating linearized system at the equillibrium point and finding its
% eigenvalues to check for stability

%% Cleanup
clear
clc
close all

%% State Space Of the System
syms x1 x2 x3 x4
syms F
syms M
syms m J l
syms c gamma
syms g

% Equillibrium State
x1_eq = 0;
x2_eq = 0;
x3_eq = 0;
x4_eq = 0;
x_eq = [x1_eq; x2_eq; x3_eq; x4_eq];

% Equillibrium Inputs
F_eq = 0;
u_eq = [F_eq];

% State Space Representation (Borrowed from problem 4)
x1dot = x2;
x2dot = (F*(J + m*l^2) - (J + m*l^2)*c*x2 - (J + m*l^2)*l*m*x4^2*sin(x3) + g*l^2*m^2*sin(2*x3)/2 - gamma*l*m*x4*cos(x3))/((J + m*l^2)*(M+m) - l^2*m^2*cos(x3)*cos(x3));
x3dot = x4;
x4dot = (F*l*m*cos(x3) + (M+m)*g*l*m*sin(x3) - (M+m)*gamma*x4 - c*l*m*x2*cos(x3) - l^2*m^2*x4^2*sin(2*x3)/2)/((J + m*l^2)*(M+m) - l^2*m^2*cos(x3)*cos(x3));

% State Space System
xdot = [x1dot x2dot x3dot x4dot];
x = [x1; x2; x3; x4];
u = [F];

%% Linearisation
% Jacobians for X and U
jacx = jacobian(xdot, x);
jacu = jacobian(xdot, u);

% Getting matrix A (at equillibrium)
A = subs(jacx, x, x_eq);
A = subs(A, u, u_eq);

% Getting matrix B (at equillibrium)
B = subs(jacu, u, u_eq);
B = subs(B, x, x_eq);

%% Evaluating A, B with given parameters

% Borrowed from problem 4
param.m = 0.2;
param.M = 0.5;
param.J = 0.006;
param.l = 0.3;
param.c = 0.1;
param.gamma = 0.1;
param.g = 9.81;

% evaluating both A and B
A_eval = eval(subs(A, {m, M, J, l, c, gamma, g}, {param.m, param.M, param.J, param.l, param.c, param.gamma, param.g}))
B_eval = eval(subs(B, {m, M, J, l, c, gamma, g}, {param.m, param.M, param.J, param.l, param.c, param.gamma, param.g}))


%% Compute Eigenvalues
[V, D] = eig(A_eval)

% If any eigenvalue > 0, the system is unstable.