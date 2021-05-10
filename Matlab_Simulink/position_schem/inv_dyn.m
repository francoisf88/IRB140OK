
function y = inv_dyn(u)


q = u(1:6)'; %mapping position
dq = u(7:12)'; %mapping velocity
alp = u(13:18); %mapping input


Abb = Abb_model(); % call function robot model 

%%% create dynamic model
B = Abb.inertia(q);
C = Abb.coriolis(q,dq);
g_vector = Abb.gravload(q,[0 0 9.81])';

y = B*alp + C*dq' + g_vector; %% control law u = inv(B)y + eta