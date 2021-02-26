clc
clear all
close all

%% Manipulator's parameteters
Link_Lengths = ([180,180,210,210,200,200,126])/1000;
q = [0,0,0,0,0,0,0]';


%% Manipultor moving along a square
for i = [-0.6:0.1:0.6]
    Desired_p = [i;-0.4;0;0;pi/2;0];
    q = IK_TP(q,Desired_p);
    robot(q,Link_Lengths);
end

for i = [-0.4:0.1:0.4]
    Desired_p = [0.6;i;0;0;pi/2;0];
    q = IK_TP(q,Desired_p);
    robot(q,Link_Lengths);
end

for i = [0.6:-0.1:-0.6]
    Desired_p = [i;0.4;0;0;pi/2;0];
    q = IK_TP(q,Desired_p);
    robot(q,Link_Lengths);
end



for i = [0.4:-0.1:-0.4]
    Desired_p = [-0.6;i;0;0;pi/2;0];
    q = IK_TP(q,Desired_p);
    robot(q,Link_Lengths);
end


%% IK
function q = IK_TP(q,desired_p)
p = FK(q);
eulZYZ = rotm2eul(p(1:3,1:3),'ZYZ');
p = [p(1:3,4);eulZYZ'];

e = desired_p -p
r1 = e(1:3)
r2 = e(4:5)
while abs(e(1))>5e-2||abs(e(2))>5e-2||abs(e(3))>5e-2
        J = jacobian_(q);
        J1 = J(1:3,:);
        J2 = J(4:5,:);
        P1 = eye(7,7)-pinv(J1)*J1;
        
        del_q = pinv(J1)*r1 + pinv(J2*P1)*(r2-J2*pinv(J1)*r1)
        q = q + del_q;
        p = FK(q);
        eulZYZ = rotm2eul(p(1:3,1:3),'ZYZ');
        p = [p(1:3,4);eulZYZ'];
        e = desired_p - p
        r1 = e(1:3);
        r2 = e(4:5);
end

end

