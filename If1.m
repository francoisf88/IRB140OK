function y = If1(u)
x = u(1:6);
time = u(7:end);


if time(end)>10
    y = x;
else
    y = zeros(size(x));
end
end

