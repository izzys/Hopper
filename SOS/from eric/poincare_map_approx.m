clc; close all; clear all;

%% fit polynomal for each row of x_new = f(x_old)
pvar x1 x2
Dat = load('poincare_sample1.mat');
Xs = Dat.Xs;
Ys = Dat.Ys;
LC = Dat.LC;
A = Dat.A;
A = A([2,4],[2,4]);
Xs = Xs - repmat([LC(2) LC(4)],length(Xs),1);
Ys = Ys - repmat([LC(2) LC(4)],length(Xs),1);
Ylin = A*Xs';
Ylin = Ylin';
Ynl = Ys - Ylin;
%%
X = monomials([x1;x2],[1:25]);
C = mpvar('c',[1,length(X)]);
p = C*X;

[pfit1,cfit1,info1] = pdatafit(p,[x1;x2],Xs',Ynl(:,1));
[pfit2,cfit2,info2] = pdatafit(p,[x1;x2],Xs',Ynl(:,2));

F = [pfit1;pfit2] + A*[x1;x2];
[A,f0] = plinearize(F,[x1;x2],[0;0]);

%%
Yapp = double(subs(F,[x1;x2],Xs'))';

X_grid = reshape(Xs(:,1),15,11);
Y_grid = reshape(Xs(:,2),15,11);
Z1 = reshape(Ys(:,1),15,11);
Z2 = reshape(Ys(:,2),15,11);
Z1lin = reshape(Yapp(:,1),15,11);
Z2lin = reshape(Yapp(:,2),15,11);
figure()
mesh(X_grid,Y_grid,Z1)
xlabel('x_2(k)')
ylabel('dx_2(k)')
zlabel('x_2(k+1)')
hold on
plot3(0,0,0,'+')
mesh(X_grid,Y_grid,Z1lin)
figure()
mesh(X_grid,Y_grid,Z2)
xlabel('x_2(k)')
ylabel('dx_2(k)')
zlabel('dx_2(k+1)')
hold on
plot3(0,0,0,'+')    
mesh(X_grid,Y_grid,Z2lin)

%% scale and save:
scale = eye(2)*0.1;
F = inv(scale)*subs(F,[x1;x2],scale*[x1;x2])
save('Aprox.mat','F','scale')