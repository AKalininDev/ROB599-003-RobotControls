%% ROB699_HW6_P2
% Analyzing LQR controller of a spring-mass-damper system

%% Clean Up
clear
clc
close all

%% Define the System

% Define System Variables
syms m k b
syms x dx ddx u

% Define the Numerical Values of the System Parameters
m_val = 1; % kg
b_val = 2; % Ns/m
k_val = 1; % N/m

% System Dynamics Equation
eq = m*ddx + b*dx + k*x == u;

% Solve for the Acceleration
ddx = solve(eq, ddx);
ddx = collect(ddx, [x, dx, u])

% State Space Representation of the System
A = [0, 1; -k/m, -b/m];
B = [0; 1/m];

% Substitude numerical values for the system parameters
A_val = eval(subs(A, [m, b, k], [m_val, b_val, k_val]));
B_val = eval(subs(B, [m, b, k], [m_val, b_val, k_val]));
% dx = A_val*x + B_val*u

%% Problem 2A: Simulate controller u = -Kx, K = [5, 1]

% Define K
K = [5, 1];

% Define State Space Representation of the Closed Loop System
A_cl = A_val - B_val*K; % dx = A_val*x - B_val*K*x = (A_val - B_val*K)*x

% Define Simulation Parameters
x_o = [1; 0]; % x(0) = 1, dx(0) = 0
t_span = [0 10]

% Simulate the Closed Loop System
[t, x] = ode45(@(t, x) A_cl*x, t_span, x_o);

% Evaluate the dx(t) from the Simulation Results
dx = x * A_cl'; % dx = (A_cl*x')' = x*A_cl' to match the dimensions of x

% Plot the Simulation Results
% Create figure with specific size
fig = figure('Position', [100, 100, 1900, 1400]);

% First subplot
subplot(2, 1, 1)
plot(t, x(:, 1), 'LineWidth', 3, 'Color', '#0072BD')
hold on
plot(t, x(:, 2), 'LineWidth', 3, 'Color', '#D95319')
xlabel('Time [s]', 'FontSize', 16)
ylabel('$$\displaystyle{x}$$', 'Interpreter', 'latex', 'FontSize', 24, 'FontWeight', 'bold')
title('Position and Velocity', 'FontSize', 24)
legend('$x_1$ [m]', '$x_2$ [m/s]', 'Interpreter', 'latex', 'FontSize', 20, 'Location', 'best')
grid on
grid minor
set(gca, 'FontSize', 14)
hold off

% Second subplot
subplot(2, 1, 2)
plot(t, dx(:, 1), 'LineWidth', 3, 'Color', '#0072BD')
hold on
plot(t, dx(:, 2), 'LineWidth', 3, 'Color', '#D95319')
xlabel('Time [s]', 'FontSize', 16)
ylabel('$$\displaystyle{\dot{x}}$$', 'Interpreter', 'latex', 'FontSize', 24, 'FontWeight', 'bold')
title('Velocity and Acceleration', 'FontSize', 24)
legend('$\dot{x}_1$ [m/s]', '$\dot{x}_2$ [m/s$^2$]', 'Interpreter', 'latex', 'FontSize', 20, 'Location', 'best')
grid on
grid minor
set(gca, 'FontSize', 14)
hold off

sgtitle('Figure 4. Simulation of the Mass-Spring-Damper System $$\displaystyle{\dot{x} = Ax + Bu}$$ with Controller $$u = -Kx$$, $$K = [5, 1]$$', 'Interpreter', 'latex', 'FontSize', 24, 'FontWeight', 'bold')
saveas(fig, 'Figure 4. Mass Spring Damper. 2A.png', 'png')

