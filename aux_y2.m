function y = aux_y2(u)

x = u(1:6); %mapping position
x_d = u(7:12); %mapping desired position
dq = u(13:18); %mapping velocity
dx_d = u(19:24); %mapping desired velocity
ddx_d = u(25:30); %mapping desired acceleration
q = u(31:36);
 
Abb = Abb_model(); % call function robot model 

 

Ja = Abb.jacob0(q,'rpy');
dJa = Abb.jacob_dot(q,dq);
invJa = pinv(Ja);

dx = Ja*dq;

dx_tilda = dx_d - dx; %error position
x_tilda = x_d - x; %error velocity

Kp = 1e-5*diag([1 1 1 1 1 1]); %weigth matrix position
Kd = 3e-6*diag([1 1 1 1 1 1]); %weigth matrix velocity 

y = invJa*(ddx_d + Kd*dx_tilda + Kp*x_tilda - dJa); %add control inputs

