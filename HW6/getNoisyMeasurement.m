%% Function that Generates a Noisy Measurement Given a State an Noize Vector
% Linearly interpolates the noize vector to get the noize at the current time

function y = getNoisyMeasurement(t, x, C, generated_noize, t_span)
z = interp1(t_span, generated_noize', t, "linear")';
y = C*x + z;
end