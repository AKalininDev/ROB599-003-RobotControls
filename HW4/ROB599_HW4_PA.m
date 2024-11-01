%% ROB599_HW4_P1_A
% Simiulating a PD Controller with Motor Angle Feedback for Series Elastic Actuator.

%% Clean Up
clear
clc

%% Define System Variables

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


%% Define EOMs
eq1 = Jm*theta_m_ddot + Bm*theta_m_dot - k*(theta_l - theta_m) == u
eq2 = Jl*theta_l_ddot + Bl*theta_l_dot + k*(theta_l - theta_m) == 0
eq3 = u == kp*(theta_cmd - theta_m) - kd*theta_m_dot

%% Define State Space

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
x2_dot = collect(x2_dot, [x1, x2, x3, x4, theta_cmd])

% Solve for x4_dot
x4_dot = solve(eq2_state, x4_dot);
x4_dot = simplify(x4_dot);
x4_dot = collect(x4_dot, [x1, x2, x3, x4, theta_cmd])

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