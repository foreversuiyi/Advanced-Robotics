function tau = controlLaw_ComputedTorque(t,model,x)

    q  = x(1:2,:);
    qd = x(3:4,:);
    [des_q, des_qd, des_qdd] = desiredJointTrajectory(t);
    Kd = diag([20, 30]);
    Kp = diag([100, 125]);
    
    y = des_qdd + Kd*(des_qd - qd) + Kp*(des_q - q);
    
    [M, C, G, F] = RRPlanarManipulatorEquation(model, x);
    % put own algorithm here, tau will be the output of your controller
    tau = M*y + C*qd + G + F*qd;
%     tau = M*y + G + F*qd;
%     tau = 2*M*y + C*qd + G + F*qd;

end