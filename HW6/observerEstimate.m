%% Function observerEstimate that Computes the Observer Estimate of the State Space Derivative

function x_hat_dot = observerEstimate(x_hat, A, B, K, L, C, y_sense)
x_hat_dot = (A - B*K)*x_hat + L*(y_sense - C*x_hat);
end