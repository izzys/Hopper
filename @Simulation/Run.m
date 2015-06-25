function Sim = Run(Sim)

                    % first closed-loop control action::
                    Sim.Out.delta_actual = Sim.IC(Sim.ModCo)'-Sim.LC(Sim.ModCo)';  
                    %d = Sim.Out.delta_actual
              %      Sim.Out.delta_linear = Sim.Out.delta_actual;
                    Sim.Con.del_dx = -Sim.Con.Kcontroller*Sim.Out.delta_actual;


if Sim.WaitbarFlag
waitbar_h = waitbar(0,'Simulation running...');
end

if Sim.Graphics == 1
    options=odeset('MaxStep',Sim.tstep/10,'RelTol',1e-7,'AbsTol',1e-7,...
        'OutputFcn', @Sim.RealTimePlot, 'Events', @Sim.Events);
else
    options=odeset('MaxStep',Sim.tstep/10,'RelTol',1e-7,'AbsTol',1e-7,...
        'Events', @Sim.Events);
end
[TTemp,XTemp,TE,YE,IE]=ode45(@Sim.Derivative,Sim.tspan,Sim.IC,options);

T=TTemp;
X=XTemp;

while TTemp(end)<Sim.tspan(end-1) && Sim.StopSim==0
   
     LiftOff = 0;
   %  Sim.RecordEvents(TE,YE,IE);
     NormalForce = Sim.Mod.spr_k*(Sim.Mod.GetSpringLength(X(end,Sim.ModCo))-Sim.Mod.spr_l0)+Sim.Mod.damping*X(end,4)-Sim.Mod.m1*Sim.Mod.g;
          
     for i=1:size(IE,1)
         switch IE(i)     
            case 1 % Lower mass touched ground 
                              
                if Sim.Mod.OnGround  
                    Sim.newIC=[ Sim.Mod.l1/2  , X(end,2),0,X(end,4),X(end,Sim.ConCo)];
                else   

                    Sim.Mod.dx=Sim.Con.Command(X(end,Sim.ModCo),Sim.Mod);
                     
                    if NormalForce<=0
                      Sim.Mod.OnGround=1;
                      
                      Time_at_TD = TTemp(end);
                    end

                    Sim.newIC=[ Sim.Mod.l1/2  , X(end,2),0,X(end,4),X(end,Sim.ConCo)] ;
                    state_after_TD = Sim.newIC;
                end
                
                if Sim.Mod.will_hop_or_not(Sim.newIC)<0
                    Sim.Out.Type = Sim.EndFlag_WillNotHop;
                    Sim.Out.Text = 'State entered no-hop-ellipse. Will not hop';
                    Sim.StopSim = 1; 
                end
                
             case 2 % Lower mass detached
                 
                if Sim.Mod.OnGround    
                         
                    Sim.newIC = [X(end,Sim.ModCo), X(end,Sim.ConCo)];
                    LiftOff = 1;                                       
                else       
                    Sim.newIC = [X(end,Sim.ModCo), X(end,Sim.ConCo)];
                end 
                
                 
             case 3 % Will not hop
                 
                        Sim.Out.Type = Sim.EndFlag_WillNotHop;
                        Sim.Out.Text = 'State entered no-hop-ellipse. Will not hop';
                        Sim.StopSim = 1;         
                 
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
        
                if Sim.EndCond(1) == 1 && Sim.Out.Type ~= Sim.EndFlag_WillNotHop
                    if Sim.StepsTaken >= Sim.EndCond(2) 
                        Sim.Out.Type = Sim.EndFlag_DoneNumberOfSteps;
                        Sim.Out.Text = ['Finished taking ',num2str(Sim.StepsTaken),...
                            ' steps'];
                        Sim.StopSim = 1;
                    end        
                end
                
       Sim.Con.LiftOffOccured = 1;
                     

       
                    %closed-loop control action:
         
                    Sim.Out.delta_actual(:,Sim.StepsTaken+1) = Sim.newIC(Sim.ModCo)'-Sim.LC';
                    %d = Sim.Out.delta_actual(:,Sim.StepsTaken+1);
                    Sim.Con.del_dx = -Sim.Con.Kcontroller*Sim.Out.delta_actual(:,Sim.StepsTaken+1);     
                   % Sim.Out.delta_linear(:,Sim.StepsTaken+1) =Sim.Px_u*Sim.Out.delta_linear(:,Sim.StepsTaken);%+Sim.Ba*Sim.Con.Kcontroller);
                 %   d1 = Sim.Out.delta_linear(:,Sim.StepsTaken+1)
                  %  Sim.Out.delta_linear(:,Sim.StepsTaken+1) =Sim.Px*Sim.Out.delta_linear(:,Sim.StepsTaken)-Sim.Bu*Sim.Con.Kcontroller*Sim.Out.delta_linear(:,Sim.StepsTaken);
                %    d2 = Sim.Out.delta_linear(:,Sim.StepsTaken+1)
                   % d_linear_by_px = Sim.Out.delta_linear(:,Sim.StepsTaken+1);
                    
                 %   d_actual = Sim.Out.delta_actual(:,Sim.StepsTaken+1)
                    
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
% Sim.Out


