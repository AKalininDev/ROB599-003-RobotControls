syms kp kd l

a = [0, 1; 24.525-kp*12.5, -0.333 * 12.5- kd*12.5]
il = [l, 0; 0, l]
eval(det(a -il))