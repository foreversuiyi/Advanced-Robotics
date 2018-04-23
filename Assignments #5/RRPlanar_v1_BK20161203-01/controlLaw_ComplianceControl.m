function tau = controlLaw_ComplianceControl(t,model,x)

    q  = x(1:2,:);
    qd = x(3:4,:);
    Fe = 10;
    [~, cur_x] = RRPlanarFKine(model,q);
    Jac = RRPlanarJacobian(model,q);
    cur_xd = Jac*qd;
    
    [des_x, des_xd] = desiredOperTrajectory(t);
    delta_x = des_x - cur_x;
    delta_xd = des_xd - cur_xd;
    
    [~, ~, G, F] = RRPlanarManipulatorEquation(model, x);
    % put own algorithm here, tau will be the output of your controller
    if des_x(1) < 1.2
        Kp = diag([10000, 20000]);
        Kd = Kp/10;
        tau = G + F*qd + Jac'*(Kp*delta_x + Kd*delta_xd);
    else
        Kp = diag([Fe/(des_x(1) - 1), 2000]);
        Kd = Kp/10;
        tau = G + F*qd + Jac'*Kp*delta_x - Jac'*Kd*Jac*qd - Jac'*[-Fe; 0];
    end
    
end