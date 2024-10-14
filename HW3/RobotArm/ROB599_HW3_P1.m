%% ROB599_HW3_P1 
% Evaluating the Model for q1 and q2
%% Cleanup
clear
clc
close all
%% Define Symbolic Parameters

% Parameters

% Inertia Terms
syms m1 m2
syms I1 I2

% Geometry Terms
syms l1 l2
syms lc1 lc2

% State Variables
syms q1 q2
syms q1dot q2dot
syms q1_dot_dot q2_dot_dot

% Disturbances
syms tau1 tau2

% Fundamental Constants
syms g

%% Define the Model

% Mass/inertia matrix
M(1,1) = m1*lc1^2 + m2*(l1^2 + lc2^2 + 2*l1*lc2*cos(q2))+I1+I2; 
M(1,2) = m2*(lc2^2+l1*lc2*cos(q2))+I2;
M(2,1) = M(1,2);
M(2,2) = m2*lc2^2+I2;

% Coriolis matrix
C(1, 1) = -m2*l1*lc2*sin(q2)*q2dot;
C(1, 2) = -m2*l1*lc2*sin(q2)*(q1dot + q2dot);
C(2, 1) = m2*l1*lc2*sin(q2)*q1dot;
C(2, 2) = 0;

% Gravity terms
N(1) = m1*g*lc1*cos(q1) + m2*g*(l1*cos(q1) + lc2*cos(q1 + q2));
N(2) = m2*g*lc2*cos(q1 + q2);

U = [tau2; tau2];

%% Define Numerical Parameters
% Inertia Terms
param.m1 = 7.848;
param.m2 = 4.49;

param.I1 = 0.176;
param.I2 = 0.0411;

% Geometry Terms
param.l1 = 0.3;
param.lc1 = 0.1554;
param.lc2 = 0.0341;

% Fundamental Constants
param.g = 9.81;

%% Evaluate the Model

M_eval = subs(M, {m1, m2, I1, I2, l1, lc1, lc2, g}, ...
    {param.m1, param.m2, param.I1, param.I2, param.l1, param.lc1, param.lc2, param.g})

C_eval = subs(C, {m1, m2, I1, I2, l1, lc1, lc2, g}, ...
    {param.m1, param.m2, param.I1, param.I2, param.l1, param.lc1, param.lc2, param.g})

N_eval = subs(N, {m1, m2, I1, I2, l1, lc1, lc2, g}, ...
    {param.m1, param.m2, param.I1, param.I2, param.l1, param.lc1, param.lc2, param.g})

U_eval = subs(U, {m1, m2, I1, I2, l1, lc1, lc2, g}, ...
    {param.m1, param.m2, param.I1, param.I2, param.l1, param.lc1, param.lc2, param.g})

%% Compute q1_dot_dot and q2_dot_dot

q1_dot_dot = 1/det(M) * [M(2,2), -M(2,1)]*(U - C * [q1dot; q2dot] - N);

q2_dot_dot = 1/det(M) * [M(1,1), -M(1,2)]*(U - C * [q1dot; q2dot] - N_eval);


%% Evaluate q1_dot_dot and q2_dot_dot

q1_dot_dot_eval = vpa(subs(expand(q1_dot_dot), {m1, m2, I1, I2, l1, lc1, lc2, g}, ...
    {param.m1, param.m2, param.I1, param.I2, param.l1, param.lc1, param.lc2, param.g}))

q2_dot_dot_eval = vpa(subs(expand(q2_dot_dot), {m1, m2, I1, I2, l1, lc1, lc2, g}, ...
    {param.m1, param.m2, param.I1, param.I2, param.l1, param.lc1, param.lc2, param.g}))