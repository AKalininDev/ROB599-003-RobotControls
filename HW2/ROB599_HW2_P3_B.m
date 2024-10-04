%% ROB599-HW2-Problem-3B
% Simulating inverted pendulum controller using a 
% linearised model (linearized only @equillibrium x1=0, x2=0.
%% Cleanup
clear
clc
close all

%% System Parameters (SI)

% Intertial
param.m = 0.2;      %kg
param.J = 0.006;    %kgm2
param.l = 0.3;      %m

% Non-Conservative
param.gamma = 0.1; %N*m*sec/rad

% Fundamental
param.g = 9.81;    %N/kg

% Disturmabces
param.w = 3;       %rad/sec

% Controller Gains
param.kp = 5;
param.ki = 2.3562
param.kd = 0.1;


%% Evaluating Numerical A and B

% State Matrix
a_1 = [0, 1, 0];
a_2 = [(1/(param.J + param.m * param.l^2)) * ((param.m*param.g - param.kp)*param.l), ...
     - (param.gamma + param.l*param.kd) * (1/(param.J + param.m * param.l^2)), ...
     (1/(param.J + param.m * param.l^2))*param.l*param.ki];
a_3 = [-1, 0, 0];


A = [a_1; a_2; a_3]


% Input Matrix
b_1 = [0, 0]; % assuming no disturbance
b_2 = [(1/(param.J + param.m * param.l^2)) * param.kp*param.l, (1/(param.J + param.m * param.l^2)) * param.kd*param.l];
b_3 = [1, 0];

B = [b_1; b_2; b_3]

%% Checking Stability of the System

[V, D] = eig(A);
D 


%% Solve for x(t->inf) (Stable Linearized System)

X_final = -inv(A) * B

% Evaluating the final values
X_final_eval = -inv(A) * B * [pi/3; param.w]


%% Simulation Parameters
to = 0;
tFinal = 10;

% Generating the time vector
tSim = to:0.01:tFinal;

% Initial Conditions
xo = [pi/10; 0.3;0];

% Input Signal (Constant)
theta_command = systemInput(tSim) * ones(size(tSim));

%% Run Simulation
[tOut, xOut] = ode45(@(t,x) LinearizedInvertedPendulumFB(t, x, param), tSim, xo);

%% Display the Results
figure('Position', [100, 100, 1200, 900]);  % Width: 1200, Height: 900


sgtitle('Figure 2. Linearized Inverted Pendulum. Feedback Controller. Kp = 5, Ki = 2.3562, Kd = 0.1',...
    'FontSize', 24, 'FontWeight', 'bold');
 
% Plot Pendulum Angle Command Signal    
subplot(4,1,1);
plot(tSim, theta_command, 'LineWidth', 2.5, 'Color', '#0072BD');
ylabel('\fontsize{12}{16}\textbf{Input $\theta_{\mathrm{cmd}}$ (rad)}', 'FontSize', 18, ...
    'FontWeight', 'bold', 'Interpreter', 'latex');
set(gca, 'FontSize', 14, 'Box', 'off', 'LineWidth', 1.5, 'FontName', 'Helvetica');
grid on;
grid minor;

% Plot Pendulum Angular Position
subplot(4,1,2);
plot(tOut, xOut(:,1), 'LineWidth', 2.5, 'Color', '#D95319');
ylabel('\fontsize{12}{16}\textbf{x1 = ${\theta}$ (rad)}', 'FontSize', 18, ...
    'FontWeight', 'bold', 'Interpreter', 'latex');
set(gca, 'FontSize', 14, 'Box', 'off', 'LineWidth', 1.5, 'FontName', 'Helvetica');
grid on;
grid minor;

% Plot Pendulum Angular Velocity
subplot(4,1,3);
plot(tOut, xOut(:,2), 'LineWidth', 2.5, 'Color', '#EDB120');
ylabel('\fontsize{12}{16}\textbf{x2 = $\dot{\theta}$ (rad/s)}', 'FontSize', 18, ...
    'FontWeight', 'bold', 'Interpreter', 'latex');
set(gca, 'FontSize', 14, 'Box', 'off', 'LineWidth', 1.5, 'FontName', 'Helvetica');
grid on;
grid minor;


% Plot Pendulum Angular Velocity
subplot(4,1,4);
plot(tOut, xOut(:,3), 'LineWidth', 2.5, 'Color', 'c');
ylabel('\fontsize{12}{16}\textbf{x3 = $\int \theta_{\mathrm{err}}\, dt$ (rad$\cdot$s)}', 'FontSize', 18, ...
    'FontWeight', 'bold', 'Interpreter', 'latex');
set(gca, 'FontSize', 14, 'Box', 'off', 'LineWidth', 1.5, 'FontName', 'Helvetica');
xlabel('Time (s)', 'FontSize', 18, 'FontWeight', 'bold', 'FontName', 'Helvetica');
grid on;
grid minor;

% Reduce Margins
set(gca, 'LooseInset', max(get(gca, 'TightInset'), 0.02));

print(gcf, 'ROB599-HW#2-Problem_3B-Inverted_Pendulum_PID_Control.png', ...
    '-dpng', '-r300'); 


%% Target Angle Calculator
function theta_comand = systemInput(t)
    theta_comand = pi/3;
end

%% State Space Of the System (Linearized)
function xdot = LinearizedInvertedPendulumFB(t, x, param)

    % State variables
    x1 = x(1);
    x2 = x(2);
    x3 = x(3);
    theta_command = systemInput(t);
   
    x1dot = x2;

    x2dot = (1/(param.J + param.m * param.l^2))* ((param.m*param.g - param.kp)*param.l * x1 ...
        - (param.gamma + param.l*param.kd)*x2 ...
        + param.l*param.ki*x3...
        + param.kp*param.l*theta_command + param.kd*param.l*param.w);

    x3dot = -x1 + theta_command;


    xdot = [x1dot; x2dot;x3dot];

end
