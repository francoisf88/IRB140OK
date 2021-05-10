
function y = dir_kin(u)

q = u';


abb = Abb_model();
T = abb.fkine(q);
[r,pos] = tr2rt(T);
kk(1) = pos(1);
kk(2) = pos(2);
kk(3) = pos(3);

rpy = tr2rpy(T,'xyz');
kk(4) = rpy(1);
kk(5) = rpy(2);
kk(6) = rpy(3);


y = kk; 