function y = Inverse_Jac(u)
q = u(1:6)'; 
x = u(7:9); 


Abb = Abb_model(); % call function robot model 

Ja = Abb.jacob0(q);
y = pinv(Ja(1:3,:))*x;

end

