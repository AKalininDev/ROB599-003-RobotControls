%% Manipulator EoM

function [xdot, tau]  = manipulator(t, x, controller, param, ref)
    
    % Interpreting State Space
    q1dot = x(2); q2dot = x(4);
    q_dot = [q1dot; q2dot];
    
    % Mass/inertia matrix
    M = computeManipulatorInertiaM(x, param);

    % Coriolis matrix
    C = computeManipulatorCoriolisC(x, param);

    % Gravity terms
    N = computeManipulatorGravityN(x, param);
    
    % Compute Controller Input
    tau = controller(t, x, param, ref);
    
    % Compute the derivative terms for velocities
    q_dd = M \ (tau - C*q_dot - N);

    xdot = [x(2); q_dd(1); x(4); q_dd(2)];
end