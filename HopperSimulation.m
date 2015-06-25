%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% 1D Hopper simulation
%
% Izzy 6/11/14
%
% to fix: 1) force plot -   match dx in or out.. 
%         2) make auto Px generation per desired LC
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all;clc;clear classes;%close all
set(0,'DefaultFigureWindowStyle','normal')

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
Sim = Sim.Set('Graphics',1,'WindowLeftLocation',0.1);

% ----------------- Simulation ----------------- %
% stable LC: [0.05 0.819726570540994 0 1.117714729518736 0]';Sim.Mod.Set('m1',0.1,'m2',0.5,'spr_k',500,'damping',1);Sim.Con.Set('dx',0.02);
% not stable LC: [0.05 0.6 0 0 0]';Sim.Mod.Set('m1',1,'m2',2,'spr_k',500,'damping',1);Sim.Con.Set('dx',0.1);

c = Sim.Mod.damping;
k = Sim.Mod.spr_k;
l0 = Sim.Mod.SprL0()+Sim.Mod.l1/2+Sim.Con.dx;
m1 = Sim.Mod.m1;
g = Sim.Mod.g;

a = -c/k;
b = (m1*g+k*l0)/k;

x2d = 1.117712605880906;

  
 
  
Sim.LC = [0.05 , a*1.117712605880906+b  , 0 , 1.117712605880906]; %Z_TD_plus = Sim.IC = [0.05 0.792869753592505 0 -1.141331471889267 0];
Sim.IC = [0.05 , a*x2d+b-0.021614137365055  , 0 , x2d-0.035272727272727  , 0 ];%[Sim.LC 0];%
%Sim.IC = [0.05 , 0.82  , 0 , 0.1177  , 0 ];
tic

Sim.Init();
Sim.Run();

if Sim.Out.Type == Sim.EndFlag_Converged || Sim.Out.Type == Sim.EndFlag_TimeEndedBeforeConverge
[EigVal,EigVec] = Sim.Poincare();
EigVal
end

toc
