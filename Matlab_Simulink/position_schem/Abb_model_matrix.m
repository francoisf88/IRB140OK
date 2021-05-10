
function y = Abb_model_matrix(u)


q = u(1:6)'; %mapping position
dq = u(7:12)'; %mapping velocity
tao = u(13:18); %mapping input

Abb = Abb_model(); % call function robot model 

%%% create dynamic model
B = Abb.inertia(q);
C = Abb.coriolis(q,dq);
g_vector = Abb.gravload(q,[0 0 9.81])';

% \ddot(q) expression 
y = B\(-C*dq' -g_vector + tao);