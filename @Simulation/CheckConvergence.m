function [ Sim ] = CheckConvergence( Sim )
% This function looks at the initial conditions stored
% and compares them to check if the system converged
% to a limit cycle

    nPer = length(Sim.stepsSS); % number of periods to check
    Converge = zeros(1,nPer); % = 1 when IC are converging
    Checked = zeros(1,nPer); % = 1 when period p has been checked

    for p = 1:nPer
        if Checked(p)
            continue;
        end
        
        diff = norm(Sim.ICstore(:,1) - Sim.ICstore(:,1+p));
        
        if diff < Sim.minDiff
            % Set every period multiple of p as converging
            % e.g. for 1: 1, 2, 3, ...
            %      for 2: 2, 4, 6, ...
            Converge(p:p:end) = Converge(p:p:end) + 1;
            Checked(p:p:end) = 1;
        else
            Converge(p) = 0;
            Checked(p) = 1;
        end
        
             
    end
    
    Sim.stepsSS = Sim.stepsSS + Converge;
    
    % If the Sim is set to stop after convergence:
    if Sim.EndCond == 2        
        Period = find(Sim.stepsSS >= Sim.stepsReq);
        Increasing = find(Converge == 1, 1, 'first');
        if ~isempty(Period)

            if Period(1)<=Increasing
                
                % shortest period reached the required num. of steps
                Sim.Out.Type = Sim.EndFlag_Converged;
                Sim.Out.Text = ['Reached steady state limit cycle of period ', ...
                    num2str(Period(1)),' after ',num2str(Sim.StepsTaken),' steps'];
                
                T = Sim.Tarray(Sim.StepsTaken) - Sim.Tarray(Sim.StepsTaken-Period(1));
                
                Sim.Period = [Period(1), T];
                % Prepare data for Poincare computation
                Sim.IClimCyc = Sim.ICstore(:,1);
                
                Sim.StopSim = 1;
            
            end
            % else, if a lower period is still converging keep going
        end
    end
end

