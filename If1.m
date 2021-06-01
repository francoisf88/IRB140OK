function y = If1(u)
x = u(1:6);
rest = u(7:end);
xf = [ 0.0300    0.4801    0.1649    1.5708   -0.0000   -2.8798];
if x(2) == xf(2) && x(3)== xf(3)
    y = rest;
else
    y = zeros(size(rest));
end
end

