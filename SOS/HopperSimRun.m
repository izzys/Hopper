function Flag = HopperSimRun(Sim,x0)

c = Sim.Mod.damping;
k = Sim.Mod.spr_k;
l0 = Sim.Mod.SprL0()+Sim.Mod.l1/2+Sim.Con.dx;
m1 = Sim.Mod.m1;
g = Sim.Mod.g;

a = -c/k;
b = (m1*g+k*l0)/k;

x2d = 1.117712605880906;

Sim.IC = [0.05 , a*x2d+b+x0(1)  , 0 , x2d+x0(2)  , 0 ];

Sim.Init();
Sim.Run();

Flag = Sim.Out.Type;