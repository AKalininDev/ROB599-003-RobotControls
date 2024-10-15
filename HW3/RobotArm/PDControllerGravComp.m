%% PD Controller with Gravity Compensation.
% Specific for Robot Manpulator with System Described in manipulator.m

function tau = PDControllerGravComp(t, x, param, ref)
    
    % Interpreting State Space
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

    % Get Kp and Kd Matricies
    Kp = [Kp1, 0;
          0, Kp2];

    Kd = [Kd1, 0;
          0, Kd2];

    % Interpolate the Target State
    x_1_target = interp1(ref(end, :), ref(1, :), t, "nearest");
    x_3_target = interp1(ref(end, :), ref(3, :), t, "nearest");

    % Interprete Target State Space
    q1d = x_1_target;
    q2d = x_3_target;
    
    % Get Error and Derivative State Vectors
    e = [q1d - q1; q2d - q2];
    q_dot = [q1dot; q2dot];

    % Compute Gravity Compensation
    N = computeManipulatorGravityN(x, param);

    % Compute Controller Input
    tau = Kp*e - Kd*q_dot + N;

    % Safety Constrains
    tau(1) = constrain(tau(1), tau1UpperLim, tau1LowerLim);
    tau(2) = constrain(tau(2), tau2UpperLim, tau2LowerLim);

end

