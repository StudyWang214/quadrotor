function out = fhan(v1, v2, r, h)
% 最速控制综合函数
d = r * h *h;
a0 = h * v2;
y = v1 + a0;
a1 = sqrt(d * d + d*8*abs(y));
a2 = a0+sign(y)*(a1-d)/2;
fsg = (sign(y+d)-sign(y-d))/2;
a = (a0 + y)*fsg+a2*(1-fsg);
fsg2 = (sign(a+d)-sign(a-d))/2;
out = -r*(a/d)*fsg2 - r*sign(a)*(1-fsg2);

end