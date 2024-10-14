%% Manipulator EoM
% Author: Jiefu Zhang
% Date: Jan 24 24
%
% Modified: Anatolii Kalinin
% Date: Oct 13 24
%
% Input: 
% t: time point
% x: state variables, in this case x = [q1, q1dot, q2, q2dot]'
% controller: the function handle of your controller function
% para: the structure array with physical parameters of the manipulator
% ref: the reference trajectory to tracking, should be 'step' or 'cubic'
%
% Output:
% xdot: the time derivative of state varibles
%
% Example: If we want to simulate the closed-loop dynamics of the
% manipulator tracking a step trajectory with PD control, from t0 to tf,
% with initial condition x0, then use
%   
%   [tout, xout] = ode45(@(t, x) manipulator(t, x, @PDcontrol, para,
%   'step'), [t0 tf], x0);
%
% where @PDcontrol is the function handle of your PD controller function.
% In this example, you should have another function named 'PDcontrol',
% which generates the control input.
% 
%%
function [xdot, tau]  = manipulator(t, x, controller, param, ref)
    q1 = x(1); q1dot = x(2); q2 = x(3); q2dot = x(4);

    % You can also define your physical parameters here so you don't need
    % the input argument 'para'
    m1 = param.m1; m2 = param.m2; I1 = param.I1; I2 = param.I2; g = param.g;
    l1 = param.l1; lc1 = param.lc1; lc2 = param.lc2; 
    
    % Mass/inertia matrix
    D = zeros(2, 2);
    D(1,1) = m1*lc1^2 + m2*(l1^2 + lc2^2 + 2*l1*lc2*cos(q2))+I1+I2; 
    D(1,2) = m2*(lc2^2+l1*lc2*cos(q2))+I2;
    D(2,1) = D(1,2);
    D(2,2) = m2*lc2^2+I2;

    % Coriolis matrix
    C = zeros(2, 2);
    C(1, 1) = -m2*l1*lc2*sin(q2)*q2dot;
    C(1, 2) = -m2*l1*lc2*sin(q2)*(q1dot + q2dot);
    C(2, 1) = m2*l1*lc2*sin(q2)*q1dot;
    C(2, 2) = 0;

    % Gravity terms
    N = zeros(2, 1);
    N(1) = m1*g*lc1*cos(q1) + m2*g*(l1*cos(q1) + lc2*cos(q1 + q2));
    N(2) = m2*g*lc2*cos(q1 + q2);

    [tau, ~] = controller(t, x, param, ref);
     
    a1 = tau(1) - C(1, 1)*q1dot - C(1, 2)*q2dot - N(1);
    a2 = tau(2) - C(2, 1)*q1dot - N(2);

    xdot = [x(2);
            1/det(D)*(D(2,2)*a1 - D(1,2)*a2);
            x(4);
            1/det(D)*(-D(2,1)*a1 + D(1,1)*a2)]; 
end