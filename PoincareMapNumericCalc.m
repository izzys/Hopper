% poincare section numeric calculation:
clear all;clc;close all;clear classes;
set(0,'DefaultFigureWindowStyle','normal')

Sim = Simulation();
      
% ------------- Parameter definitions ---------- %

% Set model parameters:
Sim.Mod = Sim.Mod.Set('m1',0.1,'m2',0.5,'spr_k',500,'damping',1);

%Set controller parametrs:
Sim.Con = Sim.Con.Set('dx',0.02,'lqr_Q',10*eye(4),'lqr_R',1);

% Set simulation time:
Sim = Sim.SetTime(0,0.005,1);

% Set graphics parameters:
Sim = Sim.Set('Graphics',0,'WindowLeftLocation',1.1);

Sim.EndCond = [1 1];

% run for section of x2d:
x2d = 0.2:0.01:1.8;
%h = waitbar(0,['Running simulation...'  '0/' num2str(length(x2d ))]);

for j = 1:length(x2d )
close all
h = waitbar(j/length(x2d ),['Running simulation...' num2str(j) '/' num2str(length(x2d ))]);

c = Sim.Mod.damping;
k = Sim.Mod.spr_k;
l0 = Sim.Mod.SprL0()+Sim.Mod.l1/2+Sim.Con.dx;
m1 = Sim.Mod.m1;
g = Sim.Mod.g;

a = -c/k;
b = (m1*g+k*l0)/k;
Sim.IC = [0.05 , a*x2d(j)+b  , 0 ,  x2d(j) , 0 ];
Sim.LC = [0.05 , a*1.117712605880906+b  , 0 , 1.117712605880906];
Sim.Init();
Sim.Run();

x(j) = x2d(j);
if Sim.Out.Type ==1
y(j) = Sim.Out.X(end,4);
elseif Sim.Out.Type ==0
y(j) = NaN;  
end
delete(h)
end

figure
plot(x,y,'b.')
hold on
plot(x,x,'r')
grid on
xlabel('x(k)')
ylabel('x(k+1)')


for i=1:length(y)
    
    if isnan(y(i))
        plot(x(i),0,'ro')
    end
    
end