function Sim = HopperSimInit()

Sim = Simulation();

% Set EndCond to run the sim until:
Sim.EndCond = 2 ; % 0 - the end of time , [1 , #num ] - number of steps, 2 - system converges to LC
      
% ------------- Parameter definitions ---------- %

% Set model parameters:
Sim.Mod = Sim.Mod.Set('m1',0.1,'m2',0.5,'spr_k',500,'damping',1);

% Set controller parameters:
Sim.Con = Sim.Con.Set('dx',0.02,'lqr_Q',10*eye(4),'lqr_R',1);

% Set simulation time:
Sim = Sim.SetTime(0,0.005,125);

% Set graphics parameters:
Sim = Sim.Set('Graphics',0,'WindowLeftLocation',0.1);

% ------- Simulation -----------%
c = Sim.Mod.damping;
k = Sim.Mod.spr_k;
l0 = Sim.Mod.SprL0()+Sim.Mod.l1/2+Sim.Con.dx;
m1 = Sim.Mod.m1;
g = Sim.Mod.g;

a = -c/k;
b = (m1*g+k*l0)/k;

x2d = 1.117712605880906;

Sim.LC = [0.05 , a*1.117712605880906+b  , 0 , 1.117712605880906]; 
