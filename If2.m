function y = If2(u)
qedq = u(1:12);
time = u(13:end);

if time(end)<=10
    y =qedq;
else
    y = zeros(size(qedq));
end
end


