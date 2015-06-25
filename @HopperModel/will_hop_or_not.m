%finding p
function will_hop = will_hop_or_not(Mod,X)

dx = Mod.dx;
c = Mod.damping;
k = Mod.spr_k;
l0 = Mod.SprL0();
m1 = Mod.m1;
m2 = Mod.m2;
g = Mod.g;

a = (m1*g+k*l0)/k;
b = c/k;
lambda = sqrt(k/m2);
f = (k/m2)*l0-g;
f_t = f/lambda^2;

alpha = (a-f_t)/sqrt(1+b^2*lambda^2);

x2 = X(2);
x2d = X(4);
will_hop = ((x2-f_t)^2+(x2d/lambda)^2) - alpha^2;
