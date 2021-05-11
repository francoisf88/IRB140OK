function y = aux_y2(u)

q = u(1:6); %mapping position
x_d = u(7:12); %mapping desired position
dq = u(13:18); %mapping velocity
dx_d = u(19:24); %mapping desired velocity
ddx_d = u(25:30); %mapping desired acceleration

Abb = Abb_model(); % call function robot model 

%Conversion to task space
Ja = Abb.jacob0(q);
K = Abb.fkine(q);

dx_tilde = dx_d - Ja*dq; %error position
x_tilde = x_d - K.t; %error velocity

Kp = 1e3*diag([1 1 1 1 1 1]); %weigth matrix position
Kd = 3e2*diag([1 1 1 1 1 1]); %weigth matrix velocity 

Ja1 = inv(Ja);
dJa = Abb.jacob_dot(q,dq);

end y = Ja1*(Kp*x_tilde + Kd*dx_tilde + ddx_d-dJa*dq)
