function tau = controlLaw_PD(t,model,x)

    
    % Obtain the current joint position and velocity
    q  = x(1:2,:);
    qd = x(3:4,:);
    
    % Sample code for a PD controller 
    % with gravity compensation for trajetory tracking
    
    % (1) Please define your desiredJointTrajectory(t) 
    %     use the desiredJointTrajectory(t) to generate the desired
    %     trajectory
    [des_q, des_qd, ~] = desiredJointTrajectory(t);
    
    % (2) Define the diagonal matrix for Kp and Kd, 
    %     choose your own gains 
    Kp = diag([25000, 25000]);
    Kd = diag([20000, 20000]);
    
    % (3) Get the gravity term using the function
    %     RRPlanarManipulatorEquation(model, x);
    [~, ~, G, F] = RRPlanarManipulatorEquation(model, x);
    % (4) create the overall controller and apply to tau 
    tau = Kp*(des_q - q) + Kd*(des_qd - qd) + G + F*qd;
end