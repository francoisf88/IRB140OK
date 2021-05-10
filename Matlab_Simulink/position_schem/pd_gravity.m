
function y = pd_gravity(u)


q = u(1:6); %mapping position
q_des = u(7:12); %mapping desired position
dq = u(13:18); %mapping velocity


Abb = Abb_model(); % call function robot model
g_vector = Abb.gravload(q');



Kp = 1e4*diag([1 1 1 1 1 1]);
Kd = 5e3*diag([1 1 1 1 1 1]);
y = Kp*(q_des-q) - Kd*dq + g_vector';

