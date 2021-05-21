function y = inv_kin(u)
syms theta1 theta2 theta3 theta4 theta5 theta6  
syms D r11 r12 r13 r21 r22 r23 r31 r32 r33  A B
load('data_robot_DH.mat'); %%% DH parameters  (d,a)

%taking x,y and z
PXe = u(1);
PYe = u(2);
PZe = u(3);


% PXe_eq = cos(theta1)*sin(theta2+theta3)*d4 + cos(theta1)*cos(theta2)*a2 + cos(theta1)*a1 ; 
% PYe_eq = sin(theta1)*sin(theta2+theta3)*d4 + sin(theta1)*cos(theta2)*a2 + sin(theta1)*a1 ;
% PZe_eq = -cos(theta2+theta3)*d4 + sin(theta2)*a2 + d1 ;


%1
theta1 = atan(PYe/PZe); %in rad

%3
D = ((PXe+a1*cos(theta1))^2+(PYe+a1*sin(theta1))^2+(PZe-d1)^2-a2^2-d4^2)/(2*a2*d4);

theta3 = atan(+sqrt(1-D^2)/D);%elbow up configuration

%2
theta2 = atan((PZe-d1)/(sqrt((PXe+a1*cos(theta1))^2+(PYe+a1*sin(theta1))^2)))-tan(d4*sin(theta3)/(a2+d4*cos(theta3)));


%parameters definition
r11 = cos(theta1)*cos(theta2+theta3)*(cos(theta4)*cos(theta5)*cos(theta6) - sin(theta4)*sin(theta6)) - cos(theta1)*sin(theta2+theta3)*cos(theta6)*sin(theta5) + sin(theta1)*(cos(theta5)*cos(theta6)*sin(theta4) + cos(theta4)*sin(theta6));
r12 = -cos(theta1)*cos(theta2+theta3)*cos(theta4)*cos(theta5)*cos(theta6) + cos(theta1)*sin(theta2+theta3)*sin(theta5)*sin(theta6) - sin(theta1)*sin(theta4)*sin(theta6)*cos(theta5);
r13 = cos(theta1)*cos(theta2+theta3)*cos(theta4)*sin(theta5) + cos(theta1)*sin(theta2+theta3)*cos(theta5) + sin(theta1)*sin(theta4)*sin(theta5);

r23 = sin(theta1)*cos(theta2+theta3)*cos(theta4)*sin(theta5) + sin(theta1)*sin(theta2+theta3)*cos(theta5) - cos(theta1)*sin(theta4)*sin(theta5);
r21 = sin(theta1)*cos(theta2+theta3)*(cos(theta4)*cos(theta5)*cos(theta6) - sin(theta4)*sin(theta6)) - sin(theta1)*sin(theta2+theta3)*cos(theta6)*sin(theta5) - cos(theta1)*(cos(theta5)*cos(theta6)*sin(theta4) + cos(theta4)*sin(theta6));
r22 = -sin(theta1)*cos(theta2+theta3)*cos(theta4)*cos(theta5)*cos(theta6) + sin(theta1)*sin(theta2+theta3)*sin(theta5)*sin(theta6) + cos(theta1)*sin(theta4)*sin(theta6)*cos(theta5);

r33 = sin(theta5)*sin(theta2+theta3)*cos(theta4) - cos(theta2+theta3)*cos(theta5);
r31 = sin(theta2+theta3)*(cos(theta4)*cos(theta5)*cos(theta6) - sin(theta4)*sin(theta6)) + cos(theta2+theta3)*cos(theta6)*sin(theta5);
r32 = -cos(theta2+theta3)*sin(theta6)*sin(theta5) - cos(theta4)*sin(theta2+theta3)*cos(theta6)*cos(theta5);



%4
theta4 = atan((sin(theta1)*r13 - cos(theta1)*r23) / (cos(theta1)*cos(theta2 + theta3)*r13 + sin(theta1)*cos(theta2 + theta3)*r23 + sin(theta2 + theta3)*r33));

%5
theta5= atan((r13*(cos(theta1)*cos(theta4)*cos(theta2 + theta3) + sin(theta1)*sin(theta4)) + r23*(sin(theta1)*cos(theta4)*cos(theta2 + theta3) - cos(theta1)*sin(theta4)) + r33*cos(theta4)*sin(theta2 + theta3))/(r13*(cos(theta1)*sin(theta2 + theta3)) + r23*(sin(theta1)*sin(theta2 + theta3)) - r33*cos(theta2 + theta3)));


%6
A = (r11*(-cos(theta1)*sin(theta4)*cos(theta2 + theta3) + sin(theta1)*cos(theta4)) - r21*(sin(theta1)*sin(theta4)*cos(theta2 + theta3) - cos(theta1)*cos(theta4)) - r31*sin(theta4)*sin(theta2 + theta3));

B = (r12*(-cos(theta1)*sin(theta4)*cos(theta2 + theta3) + sin(theta1)*cos(theta4)) - r22*(sin(theta1)*sin(theta4)*cos(theta2 + theta3) + cos(theta1)*cos(theta4)) - r32*sin(theta4)*sin(theta2 + theta3));

theta6 = atan(A/B);




y = [theta1 theta2 theta3 theta4 theta5 theta6];
end

