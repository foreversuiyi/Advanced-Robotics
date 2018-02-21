function [link_x,link_y,Jacobian] = problem2_fkine(q)
%% Calculate Forward Kinematics
T_link = zeros(4, 4, 4);
T_link(:,:,1) = DH_link(0, 0, q(1), 0);
T_link(:,:,2) = DH_link(0, 1, q(2), 0);
T_link(:,:,3) = DH_link(0, 1, q(3), 0);
T_link(:,:,4) = DH_link(0, 1, 0, 0);
T_joint = zeros(4, 4, 4);
link_x = [];
link_y = [];
for k = 1:4
    if k == 1
        T_joint(:,:,k) = T_link(:,:,k);
    else
        T_joint(:,:,k) = T_joint(:,:,k-1)*T_link(:,:,k);
    end
    link_x(k) = T_joint(1, 4, k);
    link_y(k) = T_joint(2, 4, k);
end
%% Calculate Jacobian
Jacobian = zeros(3, 3);
for k = 1:3
    Jacobian(1:3, k) = cross(T_joint(1:3, 3, k), T_joint(1:3, 4, 4) - T_joint(1:3, 4, k));
end
Jacobian = Jacobian(1:2, :);
