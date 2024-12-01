%% Function that Generates a Noize Vector

function noize = generateNoizeVector(t_span, sigma, mean)
noize = sigma * randn(size(C,1), length(t_span)) + mean;
end