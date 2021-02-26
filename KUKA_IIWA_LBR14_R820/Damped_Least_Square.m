clc
clear all
close all

%% Manipulator's parameters
Link_Lengths = ([180,180,210,210,200,200,126])/1000;
q = [0,0,0,0,0,0,0]';

%% Manipulator trying to follow desired path
path = [-0.6:0.05:0.6];
for i = path
    Desired_p = [i;-0.4;0;0;0;0];
    [q,p,e] = IK_DLS(q,Desired_p);
    robot(q,Link_Lengths);
    obtained_p(find(path==i)) = sqrt(p(1)^2+p(2)^2+p(3)^2);
    error(find(path==i)) = sqrt(e(1)^2+e(2)^2+e(3)^2);
end
save p_dls.mat obtained_p
save error_dls.mat error

%% IK 
function [q,p,e] = IK_DLS(current_q,desired_p)
q = current_q;
p = FK(q);
eulZYZ = rotm2eul(p(1:3,1:3),'ZYZ');
p = [p(1:3,4);eulZYZ']
e = desired_p - p

while abs(e(1))>5e-2||abs(e(2))>5e-2||abs(e(3))>5e-2
    J_inverse = DLS(q);
    del_q = J_inverse*e;
    q = q + del_q;
    p = FK(q);
    eulZYZ = rotm2eul(p(1:3,1:3),'ZYZ');
    p = [p(1:3,4);eulZYZ']
    e = desired_p - p
end
    
end

%% Jacobian Inverse
function J_inverse = DLS(current_q)
J = jacobian_(current_q);
J_inverse  = J' * inv((J * J' + 0.1 ^ 2 * eye(6)));
end
