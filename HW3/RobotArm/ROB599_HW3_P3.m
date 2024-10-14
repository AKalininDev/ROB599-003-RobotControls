%% ROB599_HW3_P3
% Simiulating a PD Controller for Robot Arm.

%% Define Numerical Parameters of the System
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

% Gains
param.kp1 = 50;
param.kd1 = 10;

param.kp2 = 50;
param.kd2 = 10;

% Limits
param.tau1Max = 50;
param.tau1Min = -50;

param.tau2Max = 50;
param.tau2Min = -50;

%% Define Simulation Prameters

% Initial Conditions
t = linspace(0,4,10000)';
x0 = [0;0;0;0];
tau = [0; 0];

% Target Trajectory
% Step Response for Shoulder and Elbow
qd_start = pi/2; % start
qd_new = 0;      % new
t_step = 2;      % time of start -> new transition

q1d_vec = step(qd_start, qd_new, t, t_step);
q2d_vec = q1d_vec;

% Generate Stamped Target State Trajectory
nan_vector = nan(size(q1d_vec));
x_target = [q1d_vec, nan_vector, q2d_vec, nan_vector];
t_target = t;

stamped_x_target = [x_target, t_target]';

%% Run Simulation
[tout, xout] = ode45(@(t, x) manipulator(t, x, @PDControllerGravComp, param, ...
   stamped_x_target), t, x0);

% Compute Torques
tau_values = postComputeTorques(tout, xout, @manipulator, @PDControllerGravComp, param, stamped_x_target);

%% Postprocess Data
% Compute the Errors
error_vals_1 = computeErrors(xout(:,1), x_target(:,1));
error_vals_2 = computeErrors(xout(:,3), x_target(:,3));
error_vals =[error_vals_1, error_vals_2];

% Generate the Error Target Vector
zero_vector = getZeroVec(tout);

%% Plot the Results
figure('Position', [100, 100, 1800, 1200]);
sgtitle('Figure 2. Robot Arm. PD Controller w Gravity Comp. Kpi = 50. Kdi = 10.', ...
'FontSize', 24, 'FontWeight', 'bold');

% Defining Colors
angle_pose_color = [0.2980, 0.4471, 0.6902];
ref_color = [0, 0, 0]; 
velocity_color = [0.8359, 0.3682, 0.0784];  
torque_color = [0.4667, 0.6745, 0.1882]; 

% Plots (1,1), (1,2)
% Plot q1 and q1ref
subplot(4,2,1);
plot(tout, xout(:,1), 'Color', angle_pose_color, 'LineWidth', 2, ...
    'DisplayName', '$\mathbf{q_1}$, (rad)');
hold on;
plot(t', x_target(:,1), 'Color', ref_color, 'LineStyle', '--', ...
    'LineWidth', 2, 'DisplayName', '$\mathbf{q_{1,ref}}$, (rad)');
hold off;
setSubplotProperties(gca);
y_lbl_handle = ylabel('\textbf{$\mathbf{q_1}$, Shoulder (rad)}', ...
    'FontSize', 14, 'Interpreter', 'latex');
y_lbl_handle.Position(1) = -0.25;
y_lbl_handle.Position(2) = 0.35;

% Plot q2 and q2ref
subplot(4,2,2);
plot(tout, xout(:,3), 'Color', angle_pose_color, 'LineWidth', 2, ...
    'DisplayName', '$\mathbf{q_2}$, (rad)');
hold on;
plot(t', x_target(:,3), 'Color', ref_color, 'LineStyle', '--', ...
    'LineWidth', 2, 'DisplayName', '$\mathbf{q_{2,ref}}$, (rad)');
hold off;
setSubplotProperties(gca);
y_lbl_handle = ylabel('\textbf{$\mathbf{q_2}$, Elbow (rad)}', ...
    'FontSize', 14, 'Interpreter', 'latex');
y_lbl_handle.Position(1) = -0.25;
y_lbl_handle.Position(2) = 0.35;

% Set common y-axis limits for angle plots
[ymin, ymax] = getCommonYlim([xout(:,1); x_target(:,1)], [xout(:,3); x_target(:,2)]);
subplot(4,2,1); ylim([ymin, ymax]);
subplot(4,2,2); ylim([ymin, ymax]);


% Plots (2,1), (2,2)
% Plot q1 and q1ref
subplot(4,2,3);
plot(tout, error_vals(:,1), 'Color', angle_pose_color, 'LineWidth', 2, ...
    'DisplayName', '$\mathbf{e_1}$, (rad)');
hold on;
plot(tout, zero_vector, 'Color', ref_color, 'LineStyle', '--', ...
    'LineWidth', 2,  'HandleVisibility', 'off');
hold off;
setSubplotProperties(gca);
y_lbl_handle = ylabel('\textbf{$\mathbf{e_1}$, Shoulder Err (rad)}', ...
    'FontSize', 14, 'Interpreter', 'latex');
y_lbl_handle.Position(1) = -0.23;

% Plot q2 and q2ref
subplot(4,2,4);
plot(tout, error_vals(:,2), 'Color', angle_pose_color, 'LineWidth', 2, ...
    'DisplayName', '$\mathbf{e_2}$, (rad)');
hold on;
plot(tout, zero_vector, 'Color', ref_color, 'LineStyle', '--', ...
    'LineWidth', 2, 'HandleVisibility', 'off');
hold off;
setSubplotProperties(gca);
y_lbl_handle = ylabel('\textbf{$\mathbf{q_2}$, Elbow  Err (rad)}', ...
    'FontSize', 14, 'Interpreter', 'latex');
y_lbl_handle.Position(1) = -0.23;

% Set common y-axis limits for angle plots
[ymin, ymax] = getCommonYlim(error_vals(:,1), error_vals(:,2));
subplot(4,2,3); ylim([ymin, ymax]);
subplot(4,2,4); ylim([ymin, ymax]);


% Plots (3,1), (3,2)
% Plot q1dot
subplot(4,2,5);
plot(tout, xout(:,2), 'Color', velocity_color, 'LineWidth', 2, ...
    'DisplayName', '$\mathbf{\dot{q}_1}$, (rad/s)');
setSubplotProperties(gca);
y_lbl_handle = ylabel('\textbf{$\mathbf{\dot{q}_1}$, Shoulder Vel (rad/s)}', ...
    'FontSize', 14, 'Interpreter', 'latex');
y_lbl_handle.Position(1) = -0.23;

% Plot q2dot
subplot(4,2,6);
plot(tout, xout(:,4), 'Color', velocity_color, 'LineWidth', 2, ...
    'DisplayName', '$\mathbf{\dot{q}_2}$, (rad/s)');
setSubplotProperties(gca);
y_lbl_handle = ylabel('\textbf{$\mathbf{\dot{q}_2}$, Elbow Vel (rad/s)}', ...
    'FontSize', 14, 'Interpreter', 'latex');
y_lbl_handle.Position(1) = -0.23;

% Set common y-axis limits for angular velocity plots
[ymin, ymax] = getCommonYlim(xout(:,2), xout(:,4));
subplot(4,2,5); ylim([ymin, ymax]);
subplot(4,2,6); ylim([ymin, ymax]);


% Plots (4,1), (4,2)
% Plot tau1
subplot(4,2,7);
plot(tout, tau_values(:,1), 'Color', torque_color, 'LineWidth', 2, ...
    'DisplayName', '$\mathbf{\tau_1}$, (N$\mathbf{\cdot}$m)');   
setSubplotProperties(gca);
y_lbl_handle = ylabel('\textbf{$\mathbf{\tau_1}$, Shoulder Trq (N$\mathbf{\cdot}$m)}', ...
    'FontSize', 14, 'Interpreter', 'latex');
y_lbl_handle.Position(1) = -0.23;


% Plot tau2
subplot(4,2,8);
plot(tout, tau_values(:,2), 'Color', torque_color, 'LineWidth', 2, ...
    'DisplayName', '$\mathbf{\tau_2}$, (N$\mathbf{\cdot}$m)');
setSubplotProperties(gca);
y_lbl_handle = ylabel('\textbf{$\mathbf{\tau_2}$, Elbow Trq (N$\mathbf{\cdot}$m)}', ...
    'FontSize', 14, 'Interpreter', 'latex');
y_lbl_handle.Position(1) = -0.23;

% Set common y-axis limits for torque plots
[ymin, ymax] = getCommonYlim(tau_values(:,1), tau_values(:,2));
subplot(4,2,7); ylim([ymin, ymax]);
subplot(4,2,8); ylim([ymin, ymax]);

% Add space between subplots
set(gcf, 'Units', 'Normalized', 'OuterPosition', [0, 0.04, 1, 0.96]);

% Save the figure
print('ROB599-HW#3-Problem3.png', '-dpng', '-r300');

%% Helper Functions
% Computes the Torque Values at Give State Vector
function tau_values = postComputeTorques(tout, xout, model_ref, controller_ref, param, ref)
    
    num_steps = length(tout);
    tau_values = zeros(num_steps, 2);
    for i = 1:num_steps
        [~, tau] = model_ref(tout(i), xout(i,:)', controller_ref, param, ref);
        tau_values(i,:) = tau';
    end
end

% Computes the Error Vector
function error_vals = computeErrors(x_actual, x_ref)
    error_vals = x_ref - x_actual;
end

% Generates a Zero Vector
function zero_vector = getZeroVec(t_out)
    zero_vector = zeros(size(t_out));
end

% Returns Y-axis Limits for Two Sets of Data
function [ymin, ymax] = getCommonYlim(data1, data2)
    ymin = min(min(data1), min(data2));
    ymax = max(max(data1), max(data2));
    range = ymax - ymin;
    ymin = ymin - 0.1 * range;
    ymax = ymax + 0.1 * range;
end

% Sets Subplot Properties
function setSubplotProperties(ax)
    grid(ax, 'on');
    grid(ax, 'minor');
    set(ax, 'FontSize', 14);
    xlabel(ax, '\textbf{Time (s)}', 'FontSize', 16, 'Interpreter', 'latex');
    legend(ax, 'FontSize', 14, 'Location', 'northeast', 'Interpreter', 'latex');
end