%% PD Controller Implementation

function [tau, b] = PDController(t, x, param, ref)
    
    % State Space Vars
    q1 = x(1); q1dot = x(2); q2 = x(3); q2dot = x(4);
    
    % Import Gains
    Kp1 = param.kp1;
    Kd1 = param.kd1;
    
    Kp2 = param.kp2;
    Kd2 = param.kd2;
    
    % Import Limits
    tau1UpperLim = param.tau1Max;
    tau1LowerLim = param.tau1Min;

    tau2UpperLim = param.tau2Max;
    tau2LowerLim = param.tau2Min;

    % Interpolate the Target State
    x_1_target = interp1(ref(end, :), ref(1, :), t, "nearest");
    x_2_target = interp1(ref(end, :), ref(2, :), t, "nearest");
    x_3_target = interp1(ref(end, :), ref(3, :), t, "nearest");
    x_4_target = interp1(ref(end, :), ref(4, :), t, "nearest");

    % Interprete State Space
    q1d = x_1_target;
    q2d = x_3_target;

    % Compute Controller Input
    tau1 = Kp1 * (q1d - q1) - Kd1*q1dot;
    tau2 = Kp2 * (q2d - q2) - Kd2*q2dot;
    
    % Safety Constrains
    tau1 = constrain(tau1, tau1UpperLim, tau1LowerLim);
    tau2 = constrain(tau2, tau2UpperLim, tau2LowerLim);

    tau = [tau1; tau2];
    b = 0;

end

