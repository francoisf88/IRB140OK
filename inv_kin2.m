function  [y, Jacob]= inv_kin2(u)
syms theta1 theta2 theta3 theta4 theta5 theta6  
syms a1 a2 d1 d4
load('data_robot_DH.mat'); %%% DH parameters  (d,a)

%taking x,y and z
PXe = u(1);
PYe = u(2);
PZe = u(3);


PXe_rhs = cos(theta1)*sin(theta2+theta3)*d4 + cos(theta1)*cos(theta2)*a2 + cos(theta1)*a1 ; 
PYe_rhs = sin(theta1)*sin(theta2+theta3)*d4 + sin(theta1)*cos(theta2)*a2 + sin(theta1)*a1 ;
PZe_rhs = -cos(theta2+theta3)*d4 + sin(theta2)*a2 + d1 ;

PXe_eq = PXe == PXe_rhs;
PYe_eq = PYe == PYe_rhs;
PZe_eq = PZe == PZe_rhs;

y = solve([PXe_eq PYe_eq PZe_eq], [theta1 theta2 theta3 theta4 theta5 theta6 ])

Jacob = jacobian([PXe_eq PYe_eq PZe_eq], [theta1 theta2 theta3 theta4 theta5 theta6 ])
end

