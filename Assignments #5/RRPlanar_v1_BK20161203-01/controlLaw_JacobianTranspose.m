function tau = controlLaw_JacobianTranspose(t,model,x)

    q  = x(1:2,:);
    qd = x(3:4,:);
    
    [~, cur_x] = RRPlanarFKine(model,q);
    Jac = RRPlanarJacobian(model,q);
    cur_xd = Jac*qd;
    
    [des_x, des_xd] = desiredOperTrajectory(t);
    delta_x = des_x - cur_x;
    delta_xd = des_xd - cur_xd;
    
    Kd = diag([1000, 2000]);
    Kp = diag([10000, 20000]);
    
    [~, ~, G, F] = RRPlanarManipulatorEquation(model, x);
    % put own algorithm here, tau will be the output of your controller
    tau = G + F*qd + Jac'*(Kp*delta_x + Kd*delta_xd);

end