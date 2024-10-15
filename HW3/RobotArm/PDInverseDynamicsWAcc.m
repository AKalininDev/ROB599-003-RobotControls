% Simulates PD Controller with imperfect system modeling

function tau = PDInverseDynamicsWAcc(t, x, param, ref)
    
    % Interpreting State Space
    q1 = x(1); q1dot = x(2); q2 = x(3); q2dot = x(4);
    q = [q1; q2];
    qdot = [q1dot; q2dot];
    
    % Import Kp and Kd Gains
    Kp1 = param.kp1;
    Kp2 = param.kp2;
    Kd1 = param.kd1;
    Kd2 = param.kd2;

    Kp = [Kp1, 0;
          0, Kp2];
    Kd = [Kd1, 0;
          0, Kd2];
    
    % Import Upper and Lower Limits
    tau1UpperLim = param.tau1Max;
    tau2UpperLim = param.tau2Max;
    tau1LowerLim = param.tau1Min;
    tau2LowerLim = param.tau2Min;

    % Import Accuracy parameters
    MAccuracy = param.MAccuracy;
    CAccuracy = param.CAccuracy;
    NAccuracy = param.NAccuracy;

    % Interpolate the Target States at Given Time t
    x_1_target = interp1(ref(end, :), ref(1, :), t, "linear");
    x_2_target = interp1(ref(end, :), ref(2, :), t, "linear");
    x_3_target = interp1(ref(end, :), ref(3, :), t, "linear");
    x_4_target = interp1(ref(end, :), ref(4, :), t, "linear");
    x_5_target = interp1(ref(end, :), ref(5, :), t, "linear");
    x_6_target = interp1(ref(end, :), ref(6, :), t, "linear");

    % Interpret Target State Space
    q1d = x_1_target;
    v1d = x_2_target;
    q2d = x_3_target;
    v2d = x_4_target;
    a1d = x_5_target;
    a2d = x_6_target;

    qd = [q1d; q2d];
    vd = [v1d; v2d];
    ad = [a1d; a2d];
   
    % Compute Inertia, Coriolis, and Gravity Terms
    M = MAccuracy * computeManipulatorInertiaM(x, param);
    C = CAccuracy * computeManipulatorCoriolisC(x, param);
    N = NAccuracy * computeManipulatorGravityN(x, param);
    
    % Get Error Vectors
    e = qd - q;
    e_dot = vd - qdot;
    
    % Compute Controller Input
    aq = ad + Kp*e + Kd*e_dot;
    tau = M*aq + C*qdot + N;

    % Safety Constrains
    tau(1) = constrain(tau(1), tau1UpperLim, tau1LowerLim);
    tau(2) = constrain(tau(2), tau2UpperLim, tau2LowerLim);

end

