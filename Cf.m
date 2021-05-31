function y = Cf(u)

Kf = [1 1 1 1 1 1]';
Ki = diag([1 1 1 1 1 1]);

y = Kf+Ki*u;

end

