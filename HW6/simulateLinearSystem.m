%% Function simulateLinearSystem
% Simulates a linear system given the state-space representation and the input

function x_dot = simulateLinearSystem(x, A, B, u)
x_dot = A*x + B*u;
end