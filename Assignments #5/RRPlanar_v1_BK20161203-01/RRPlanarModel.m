function m = RRPlanarModel()

    model.l1  = 1; % length of link 1
    model.l2  = 1; % length of link 2
    model.lc1 = 0.5; % COM of link 1
    model.lc2 = 0.5; % COM of link 2
    model.m1  = 50; % mass of link 1
    model.m2  = 50; % mass of link 2
    model.I1  = 16.67; % moment of inertia of link 1
    model.I2  = 16.67; % moment of inertia of link 2
    model.b1  = 0; % dumper of joint 1
    model.b2  = 0; % dumper of joint 2
    %
    % disturbance
    %
    model.wm = 1; % disturbance in M term, 1 => no disturbance
    model.wc = 1; % disturbance in C term, 1 => no disturbance
    model.wg = 1; % disturbance in G term, 1 => no disturbance
    %
    % environment parameters
    model.g = 9.81;

    m = model;

end