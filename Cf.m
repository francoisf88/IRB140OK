function y = Cf(u)

inte = u(1:6);
prop = u(7:12);

    Kf = 0.064*diag([1 1 1 1 1 1]);
    Ki = 0.015*diag([1 1 1 1 1 1]);

y = Kf*prop+Ki*inte;

end

