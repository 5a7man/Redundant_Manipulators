clc
clear all
close all

%% Manipulator's parameters
Link_Lengths = ([180,180,210,210,200,200,126])/1000;
q = [0,0,0,0,0,0,0]';

%% Manipulator trungi to follow desired trajectory
path = [-0.6:0.05:0.6];
for i = path
    Desired_p = [i;-0.4;0;0;0;0];
    [q,p,e] = IK_NSM(q,Desired_p);
    robot(q,Link_Lengths);
    obtained_p(find(path==i)) = sqrt(p(1)^2+p(2)^2+p(3)^2);
    error(find(path==i)) = sqrt(e(1)^2+e(2)^2+e(3)^2);
end
save p_nsm.mat obtained_p
save error_nsm.mat error

%% IK
function [q,p,e] = IK_NSM(current_q,desired_p)
q = current_q;
q_prev = current_q;
p = FK(q);
eulZYZ = rotm2eul(p(1:3,1:3),'ZYZ');
p = [p(1:3,4);eulZYZ']
e = desired_p - p
while abs(e(1))>5e-2||abs(e(2))>5e-2||abs(e(3))>5e-2
    del_q = NSM(q,q_prev,e);
    q_prev = q;
    q = q + del_q;
    p = FK(q);
    eulZYZ = rotm2eul(p(1:3,1:3),'ZYZ');
    p = [p(1:3,4);eulZYZ'];
    e = desired_p - p;
end
    
end


%% Jacobian Inverse
function del_q = NSM(current_q, previous_q, e)
q0 = 0.1*(Objective_Function(current_q)-Objective_Function(previous_q));
J = jacobian_(current_q);
J_inverse = pinv(J);
del_q = J_inverse*e + (eye(7)-J_inverse*J)*q0;
end

function H = Objective_Function(q)
H = sqrt(det(jacobian_(q)*jacobian_(q)'));
end



