function robot(Joint_Angles,Link_Lengths)

cla
hold on
view(2) 
grid on
axis([-1 1 -1 1 -1 1]) 

T01 = Tz(Link_Lengths(1));
plot3([0 T01(1,4)],[0 T01(2,4)],[0 T01(3,4)],'-b','LineWidth', 1);
plot3(T01(1,4),T01(2,4),T01(3,4),'ro','MarkerSize',2,'LineWidth', 5);

T02 = Tz(Link_Lengths(1))*...
     Rz(Joint_Angles(1))*Tz(Link_Lengths(2));
plot3([T01(1,4) T02(1,4)],[T01(2,4) T02(2,4)],[T01(3,4) T02(3,4)],'-b','LineWidth', 1);
plot3(T02(1,4),T02(2,4),T02(3,4),'ro','MarkerSize',2,'LineWidth', 5);

T03 = Tz(Link_Lengths(1))*...
    Rz(Joint_Angles(1))*Tz(Link_Lengths(2))*....
    Ry(Joint_Angles(2))*Tz(Link_Lengths(3));
plot3([T02(1,4) T03(1,4)],[T02(2,4) T03(2,4)],[T02(3,4) T03(3,4)],'-b','LineWidth', 1);
plot3(T03(1,4),T03(2,4),T03(3,4),'ro','MarkerSize',2,'LineWidth', 5);


T04 = Tz(Link_Lengths(1))*...
    Rz(Joint_Angles(1))*Tz(Link_Lengths(2))*....
    Ry(Joint_Angles(2))*Tz(Link_Lengths(3))*....
    Rz(Joint_Angles(3))*Tz(Link_Lengths(4));
plot3([T03(1,4) T04(1,4)],[T03(2,4) T04(2,4)],[T03(3,4) T04(3,4)],'-b','LineWidth', 1);
plot3(T04(1,4),T04(2,4),T04(3,4),'ro','MarkerSize',2,'LineWidth', 5);



T05 = Tz(Link_Lengths(1))*...
    Rz(Joint_Angles(1))*Tz(Link_Lengths(2))*....
    Ry(Joint_Angles(2))*Tz(Link_Lengths(3))*....
    Rz(Joint_Angles(3))*Tz(Link_Lengths(4))*....
    Ry(Joint_Angles(4))*Tz(Link_Lengths(5));
plot3([T04(1,4) T05(1,4)],[T04(2,4) T05(2,4)],[T04(3,4) T05(3,4)],'-b','LineWidth', 1);
plot3(T05(1,4),T05(2,4),T05(3,4),'ro','MarkerSize',2,'LineWidth', 5);


T06 = Tz(Link_Lengths(1))*...
    Rz(Joint_Angles(1))*Tz(Link_Lengths(2))*....
    Ry(Joint_Angles(2))*Tz(Link_Lengths(3))*....
    Rz(Joint_Angles(3))*Tz(Link_Lengths(4))*....
    Ry(Joint_Angles(4))*Tz(Link_Lengths(5))*...
    Rz(Joint_Angles(5))*Tz(Link_Lengths(6));
plot3([T05(1,4) T06(1,4)],[T05(2,4) T06(2,4)],[T05(3,4) T06(3,4)],'-b','LineWidth',1);
plot3(T06(1,4),T06(2,4),T06(3,4),'ro','MarkerSize',2,'LineWidth', 5);


T07 = Tz(Link_Lengths(1))*...
    Rz(Joint_Angles(1))*Tz(Link_Lengths(2))*....
    Ry(Joint_Angles(2))*Tz(Link_Lengths(3))*....
    Rz(Joint_Angles(3))*Tz(Link_Lengths(4))*....
    Ry(Joint_Angles(4))*Tz(Link_Lengths(5))*...
    Rz(Joint_Angles(5))*Tz(Link_Lengths(6))*...
    Ry(Joint_Angles(6))*Tz(Link_Lengths(7));
plot3([T06(1,4) T07(1,4)],[T06(2,4) T07(2,4)],[T06(3,4) T07(3,4)],'-b','LineWidth',1);
plot3(T07(1,4),T07(2,4),T07(3,4),'ro','MarkerSize',2,'LineWidth', 5);


T08 = Tz(Link_Lengths(1))*...
    Rz(Joint_Angles(1))*Tz(Link_Lengths(2))*....
    Ry(Joint_Angles(2))*Tz(Link_Lengths(3))*....
    Rz(Joint_Angles(3))*Tz(Link_Lengths(4))*....
    Ry(Joint_Angles(4))*Tz(Link_Lengths(5))*...
    Rz(Joint_Angles(5))*Tz(Link_Lengths(6))*...
    Ry(Joint_Angles(6))*Tz(Link_Lengths(7))*...
    Rz(Joint_Angles(7));
plot3([T07(1,4) T08(1,4)],[T07(2,4) T08(2,4)],[T07(3,4) T08(3,4)],'-b','LineWidth',1);
plot3(T08(1,4),T08(2,4),T08(3,4),'ko','MarkerSize',2,'LineWidth', 5);


xlabel('x');
ylabel('y');
zlabel('z');
title('Manipulator Motion')
hold off

pause(0.05)
end
