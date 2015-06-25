function Sim = Run(Sim)


                    % first closed-loop control action::
                    Sim.Out.delta_actual = Sim.IC(Sim.ModCo)'-Sim.LC(Sim.ModCo)';
                    Sim.Out.delta_linear = Sim.Out.delta_actual;
                    Sim.Con.del_dx = -Sim.Con.Kcontroller*Sim.Out.delta_actual;
                  %  Sim.Mod.dx=Sim.Con.Command(Sim.IC(Sim.ModCo)',Sim.Mod);

if Sim.WaitbarFlag
waitbar_h = waitbar(0,'Simulation running...');
end

if Sim.Graphics == 1
    options=odeset('MaxStep',Sim.tstep/10,'RelTol',.5e-10,'AbsTol',.5e-10,...
        'OutputFcn', @Sim.RealTimePlot, 'Events', @Sim.Events);
else
    options=odeset('MaxStep',Sim.tstep/10,'RelTol',.5e-10,'AbsTol',.5e-10,...
        'Events', @Sim.Events);
end
[TTemp,XTemp,TE,YE,IE]=ode45(@Sim.Derivative,Sim.tspan,Sim.IC,options);

T=TTemp;
X=XTemp;

while TTemp(end)<Sim.tspan(end-1) && Sim.StopSim==0
   
     LiftOff = 0;
     Sim.RecordEvents(TE,YE,IE);
     NormalForce = Sim.Mod.spr_k*(Sim.Mod.GetSpringLength(X(end,Sim.ModCo))-Sim.Mod.spr_l0)+Sim.Mod.damping*X(end,4)-Sim.Mod.m1*Sim.Mod.g;
          
     for i=1:size(IE,1)
         switch IE(i)     
            case 1 % Lower mass touched ground 
                              
                if Sim.Mod.OnGround  
                    Sim.newIC=[ Sim.Mod.l1/2  , X(end,2),0,X(end,4),X(end,Sim.ConCo)];
                else   
                    
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    % do SM stuff: 

%                     [Aflight,Bflight] = Sim.Mod.GetABflight();
%                     xgal = [0.05 0.7928697535 -1.151911 -1.1413315];         % varies on LC!!!
%                     Fgal = Aflight*xgal'+Bflight;

                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                 
                    Sim.Mod.dx=Sim.Con.Command(X(end,Sim.ModCo),Sim.Mod);
                     
                    if NormalForce<=0
                      Sim.Mod.OnGround=1;
                    end

                    Sim.newIC=[ Sim.Mod.l1/2  , X(end,2),0,X(end,4),X(end,Sim.ConCo)] ;

                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    % do SM stuff:             
                    
%                     [Astance,Bstance] = Sim.Mod.GetABstance();
%                     xhat = [0.05 0.7928697535 0 -1.1413315];                % varies on LC!!!
% 
%                     Fhat = Astance*xhat'+Bstance;
% 
%                     
%                     R1x = [ 0 0 0 0 ; 
%                             0 1 0 0 ;
%                             0 0 0 0 ;
%                             0 0 0 1];
%                         
%                     H1x = [ 1 0 0 0 ];
% 
%                     S_TD = R1x+(Fhat-R1x*Fgal)/(H1x*Fgal)*H1x;
%                     Tflight = 0.2114701;                                    % varies on LC!!!
                    

                    
                    
                    
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

                end
                
             case 2 % Lower mass detached
                 
                if Sim.Mod.OnGround    
                    
                    
                    Sim.newIC = [X(end,Sim.ModCo), X(end,Sim.ConCo)];
                    LiftOff = 1;                                       
                else       
                    Sim.newIC = [X(end,Sim.ModCo), X(end,Sim.ConCo)];
                end 
         end
     end
     
     % Actions to do at lift-off:
     if LiftOff      
        Sim.ICstore(:,2:end) = Sim.ICstore(:,1:end-1);
        Sim.ICstore(:,1) = Sim.newIC';
        
      %  if Sim.Con.LiftOffOccured 
        Sim.StepsTaken = Sim.StepsTaken+1;
     %   end
        
        Sim.Tarray = [Sim.Tarray ; TTemp(end)];
        Sim = Sim.CheckConvergence();  
        
                if Sim.EndCond(1) == 1 
                    if Sim.StepsTaken >= Sim.EndCond(2) 
                        Sim.Out.Type = 1;
                        Sim.Out.Text = ['Finished taking ',num2str(Sim.StepsTaken),...
                            ' steps'];
                        Sim.StopSim = 1;
                    end        
                end
                
       Sim.Con.LiftOffOccured = 1;
       
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    % do SM stuff:

%                     [Astance,Bstance] = Sim.Mod.GetABstance();
%                     Tstance = 0.09369725;                                  % varies on LC!!!
%                     xgal = [0.05   0.8197265747   0   1.1177126];          % varies on LC!!!
%                     Fgal = Astance*xgal'+Bstance;
% 
%                     H2x= [ 0 , Sim.Mod.spr_k , 0 , Sim.Mod.damping];
%                     S_LO = eye(4) - Fgal*H2x/(H2x*Fgal);
%         
%                     Px = S_LO*expm(Astance*Tstance)*S_TD*expm(Aflight*Tflight)
                 
                    %closed-loop control action:
                    
                    
                    Sim.Out.delta_actual(:,Sim.StepsTaken+1) = Sim.newIC(Sim.ModCo)'-Sim.LC';
                    
                    Sim.Con.del_dx = -Sim.Con.Kcontroller*Sim.Out.delta_actual(:,Sim.StepsTaken+1);
                    

 
                    Sim.Out.delta_linear(:,Sim.StepsTaken+1) = (Sim.Px)*Sim.Out.delta_linear(:,Sim.StepsTaken);%+Sim.Ba*Sim.Con.Kcontroller

% 
%                     c = Sim.Mod.damping;
%                     k = Sim.Mod.spr_k;
%                     a = -c/k;
%                     Tax = [ 0 a 0 1]';
%                     Txa= [0 0 0 1];
%                     eig(Px);
%                     Txa*Px*Tax;
                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  
       Sim.Mod.dx=Sim.Con.Command(X(end,Sim.ModCo),Sim.Mod);
       Sim.Mod.OnGround=0;
     end
     
     % Wait bar:
     if Sim.WaitbarFlag
       switch Sim.EndCond(1)   
           case {0,2}
             waitbar(TTemp(end)/Sim.tend)
           case 1
             waitbar(Sim.StepsTaken/Sim.EndCond(2))  
       end
     end
    
    % Continue simulation...
    if Sim.StopSim
        break;
    end
    
    Sim.tspan=TTemp(end):Sim.tstep:Sim.tend;
    [TTemp,XTemp,TE,YE,IE]=ode45(@Sim.Derivative,Sim.tspan,Sim.newIC,options); 
    
    T=[T; TTemp]; %#ok<AGROW>
    X=[X; XTemp]; %#ok<AGROW>
        
end


if Sim.WaitbarFlag
  close(waitbar_h)
end


%for not stable IC:
if (Sim.EndCond(1) == 2) && (T(end)>=Sim.tend-Sim.tstep) && ~norm(Sim.stepsSS)
Sim.Out.Type = Sim.EndFlag_TimeEndedBeforeConverge;
Sim.Out.Text = 'Reached end of tspan before system converged';
Sim.IClimCyc = Sim.IC';
Sim.Period = [1, Inf];   
end


% after simulation ends, do this:
%Sim.RecordEvents(TE,YE,IE);
Sim.Out.X = X;
Sim.Out.T = T;
Sim.StopSim = 1;
if Sim.Out.Type~=Sim.EndFlag_WindowClosed && Sim.Graphics
Sim.PlotResults();
set(Sim.StopButtonObj,'String','Close Window');
end
Sim.Out


