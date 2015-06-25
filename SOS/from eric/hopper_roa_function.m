clear all; close all;
Dat = open('Aprox.mat');
pvar x1 x2
f = Dat.F;
scale = Dat.scale;
f(1) = cleanpoly(f(1),[],[0:2]);
f(2) = cleanpoly(f(2),[],[0:2]);
x = [x1,x2].';
f = inv(scale)*subs(f,x,scale*x);
%%
Alin = plinearize(f,[x1;x2],[0,0]);
P=dlyap(Alin.',eye(2));
V0 = x'*P*x;
p = V0;
opts.DegV = 3;
opts.Maxiter = 50;
opts.reltol = 0.01;
opts.SFadapt = 1;
opts.sos_opts = gsosoptions;
opts.sos_opts.minobj = -50;
opts.sos_opts.maxobj = 0;
opts.sos_opts.relbistol = 1e-4;
opts.sos_opts.absbistol = 1e-4;
opts.sos_opts.display = 'on';
opts.sos_opts.scaling = 'on';
[V,bet,gamma] = discrete_sys_ROA(f,x,V0,p,opts);