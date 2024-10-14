
% Function that Generates Step Response Vector with Given Parameters
function x_target = step(x_init, x_final, t_vec, t_step)
    x_target = x_init + stepfun(t_vec, t_step) * (x_final - x_init);

end