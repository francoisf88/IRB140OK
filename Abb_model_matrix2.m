function y = Abb_model_matrix2(u)
%For the simulink scheme

q = u(1:6)'; %mapping position
dq = u(7:12)'; %mapping velocity
t = u(13:18); %mapping input


Abb = Abb_model(); % call function robot model 

%%% create dynamic model
B = Abb.inertia(q);


% \ddot(q) expression 
y = inv(B)*t

end

