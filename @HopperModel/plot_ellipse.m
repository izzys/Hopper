%plot%finding p
function []= plot_ellipse(Mod,X)

x2_vec=X(:,2);
x2d_vec=X(:,4);
x2= 0.75:0.001:0.85;
x2d=-1:0.001:1;

dx = 0.02;
c = Mod.damping;
k = Mod.spr_k;
if Mod.OnGround
l0 = Mod.SprL0();
else
l0 = Mod.SprL0()+Mod.l1/2+dx;
end
m1 = Mod.m1;
m2 = Mod.m2;
g = Mod.g;


a = (m1*g+k*l0)/k;
b = c/k;
lambda = sqrt(k/m2);
f = (k/m2)*l0-g;
f_t = f/lambda^2;


figure
axis equal
plot(x2d_vec,x2_vec,'b')
hold on

plot(x2d,a-b*x2d,'g')

xlabel('x2d')
ylabel('x2')

alpha = (a-f_t)/sqrt(1+b^2*lambda^2);
t = 0:0.001:2*pi;
plot(-alpha*lambda*sin(lambda*t),alpha*cos(lambda*t)+f_t,'r')
legend('state','a-b*x2d','(x2-f)^2+(x2d/\lambda)^2=\alpha^2')

