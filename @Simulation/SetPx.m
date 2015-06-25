function [ Sim ] = SetPx( Sim )


% flight:
Sim.Mod.OnGround = 0;
Sim.Mod.dx = 0;


[Aflight,Bflight] = Sim.Mod.GetABflight();
xgal = [0.05 ,0.792869753593741 ,-1.151911127237779  ,-1.141331471901466];         % varies on LC!!!
Fgal = Aflight*xgal'+Bflight;
Tflight = 0.211470124295265;                                     % varies on LC!!!

%impact (TD):

Sim.Mod.dx = Sim.Con.dx;%careful.. this can mess up the simulation if not set back!
Sim.Mod.OnGround = 1;

[Astance,Bstance] = Sim.Mod.GetABstance();
xhat = [0.05 ,0.792869753593741 ,0 ,-1.141331471901466];                % varies on LC!!!

Fhat = Astance*xhat'+Bstance;


R1x = [ 0 0 0 0 ; 
        0 1 0 0 ;
        0 0 0 0 ;
        0 0 0 1];

H1x = [ 1 0 0 0 ];

S_TD = R1x+(Fhat-R1x*Fgal)/(H1x*Fgal)*H1x;
                               
% lift-off condition reached (TO):

[Astance,Bstance] = Sim.Mod.GetABstance();
Tstance = 0.093697250581270;                                  % varies on LC!!!
xgal = [0.05   0.819726574788300   0   1.117712605880906];          % varies on LC!!!
Fgal = Astance*xgal'+Bstance;

H2x= [ 0 , Sim.Mod.spr_k , 0 , Sim.Mod.damping];
S_LO = eye(4) - Fgal*H2x/(H2x*Fgal);
Sim.Astance_nom = Astance;
Sim.Bstance_nom = Bstance;
Sim.Tstance_nom = Tstance;
Sim.Px = S_LO*expm(Astance*Tstance)*S_TD*expm(Aflight*Tflight);


%calc Ba:
[As,Bs] = Sim.Mod.GetABstance();

Sim.B = [ 0 0 0 Sim.Mod.spr_k/Sim.Mod.m2]';

syms tau 
Sim.Ba = double( int(expm(As*tau),0,Tstance)*Sim.B );


%Bu:
Hu =-Sim.Mod.spr_k;
Sim.Bu = S_LO*Sim.Ba-(Fgal*Hu)/(H2x*Fgal);
            
  
Sim.Mod.dx = 0; %return dx to 0 to init properly.


end

