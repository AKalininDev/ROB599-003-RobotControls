%% Problem 1 D.
% Solving for Desired Response Parameters

%% Define the Equations for Desired Response Parameters
syms zeta wn

PO_equation = 12 == 100*exp((-1)*zeta * pi / sqrt(1 - zeta^2))
Ts_equation = 5 == 4/(zeta*wn)


%% Solve for Zeta and Wn
zeta_val = eval(solve(PO_equation, zeta));
zeta_val = zeta_val(2)

wn = solve(Ts_equation, wn);
wn = eval(subs(wn, zeta, zeta_val))

%% Define the Equations for Kp and Kd
syms kp kd

wn_equation = wn^2 == 12.5*kp - 24.525
zeta_equation = 2*zeta*wn == 4.167 + 12.5*kd

%% Solve for Kp and Kd

kp_val = eval(solve(wn_equation, kp))
kd_val = eval(subs(solve(zeta_equation, kd), {zeta, kp}, {zeta_val, kp_val}))

