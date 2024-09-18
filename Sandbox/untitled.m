% Parameters
m = 0.2;          % kg
M = 0.5;          % kg
J = 0.006;        % kg*m^2
l = 0.3;          % m
c = 0.1;          % N*s/m
gamma = 0.1;      % N*m*s/rad
g = 9.81;         % m/s^2

% Initial conditions
x0 = [0; 0; pi/8; 0];  % [x1; x2; x3; x4]

% Time span
tspan = [0 10];

% ODE function handle with parameters passed
params = struct('m', m, 'M', M, 'J', J, 'l', l, 'c', c, 'gamma', gamma, 'g', g);
ode_fun = @(t, x) segway_ode(t, x, params);

% Solve ODE
[t, x] = ode45(ode_fun, tspan, x0);

% Plot results
figure;
subplot(4,1,1);
plot(t, x(:,1));
xlabel('Time [s]');
ylabel('x_1 [m]');
title('Base Position');

subplot(4,1,2);
plot(t, x(:,2));
xlabel('Time [s]');
ylabel('x_2 [m/s]');
title('Base Velocity');

subplot(4,1,3);
plot(t, x(:,3));
xlabel('Time [s]');
ylabel('x_3 [rad]');
title('Pendulum Angle');

subplot(4,1,4);
plot(t, x(:,4));
xlabel('Time [s]');
ylabel('x_4 [rad/s]');
title('Pendulum Angular Velocity');

% ODE function
function xdot = segway_ode(t, x, params)
    % Unpack parameters
    m = params.m;
    M = params.M;
    J = params.J;
    l = params.l;
    c = params.c;
    gamma = params.gamma;
    g = params.g;

    % State variables
    x1 = x(1);  % Position of the base
    x2 = x(2);  % Velocity of the base
    x3 = x(3);  % Angle of the pendulum
    x4 = x(4);  % Angular velocity of the pendulum

    % Input u(t)
    if t <= 2.5
        u = 0.1 * cos(10 * t);
    elseif t < 5
        u = 0.5 * cos(5 * t);
    else
        u = 1.0 * cos(5 * t);
    end

    % Precompute trigonometric terms
    sin_x3 = sin(x3);
    cos_x3 = cos(x3);

    % Compute the denominator D
    D = (M + m)*(J + m * l^2) - (m * l * cos_x3)^2;

    % Compute b1 and b2
    b1 = u - c * x2 - m * l * sin_x3 * x4^2;
    b2 = m * g * l * sin_x3 - gamma * x4;

    % Compute numerators N1 and N2
    N1 = (J + m * l^2) * b1 + m * l * cos_x3 * b2;
    N2 = m * l * cos_x3 * b1 + (M + m) * b2;

    % Compute derivatives
    xdot = zeros(4,1);
    xdot(1) = x2;
    xdot(2) = N1 / D;
    xdot(3) = x4;
    xdot(4) = N2 / D;
end
