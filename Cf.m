function y = Cf(u)

inte = u(1:6);
prop = u(7:12);
x = u(13:18);
xf = [ 0.0300    0.4801    0.1649    1.5708   -0.0000   -2.8798];
if x(2) == xf(2) && x(3)== xf(3)
    Kf = 0.064*diag([1 1 1 1 1 1]);
    Ki = 0.015*diag([1 1 1 1 1 1]);
else
    Kf = zeros(6,6);
    Ki=Kf;
end

y = Kf*prop+Ki*inte;

end

