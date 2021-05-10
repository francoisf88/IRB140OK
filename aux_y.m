function y = aux_y(u)

q = u(1:6); %mapping position
q_d = u(7:12); %mapping desired position
dq = u(13:18); %mapping velocity
dq_d = u(19:24); %mapping desired velocity
ddq_d = u(25:30); %mapping desired acceleration

dq_tilda = dq_d - dq; %error position
q_tilda = q_d - q; %error velocity

Kp = 1e3*diag([1 1 1 1 1 1]); %weigth matrix position
Kd = 3e2*diag([1 1 1 1 1 1]); %weigth matrix velocity 

y = ddq_d + Kd*dq_tilda + Kp*q_tilda; %add control inputs