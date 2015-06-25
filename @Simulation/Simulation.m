classdef Simulation < handle & matlab.mixin.Copyable
    
    properties(Constant)

        % End flags:
        EndFlag_WillNotHop = -4;
        EndFlag_ICpenetrateGround = -3;
        EndFlag_WindowClosed = -2
        EndFlag_StoppedByUser = -1;
        EndFlag_EndOfTime = 0;
        EndFlag_DoneNumberOfSteps = 1;
        EndFlag_Converged = 2;
        EndFlag_TimeEndedBeforeConverge = 3;


    end
    
    properties
        
        Mod; % Model
        Con; % Controller
        Env; % Environment

        % State params:
        ModCo;ConCo;stDim;
        
        % Event params:
        nEvents; ModEv; ConEv;
        EventsCounter;
        
        % Simulation parameters:
        IC;newIC;
        LC;
        infTime;
        tstep; tstep_normal; tstep_small = [];
        tstart;
        tend; tspan;
        StopSim;
        PauseSim;
        SimSpeed;
        EndCond = 0;
        % Set EndCond to run the sim until:
        % 0 - the end of time
        % [1,numsteps] - numsteps are taken on end_slope
        % 2 - the system converges to a limit cycle
        WaitbarFlag;
        
        
        % Performance tracking / Statistics:
        Out; % output holder
        
        % Graphics:
        FigWin;
        Graphics;
        Fig;
        Once;   
        FigWidth;
        FigHeight;
        AR;
        FlMin;
        FlMax;
        HeightMin;
        HeightMax;
        StopButtonObj;
        PauseButtonObj;
        TimeDispObj;
        WindowLeftLocation;
        
        % Poincare map calculation parameters
        IClimCyc; Period;
        PMeps = 5e-7;
        PMeigs; PMeigVs;
        
        % Convergance parameters:
        StepsTaken;
        ICstore; nICsStored;
        minDiff = 1e-5; % Min. difference for LC convergence
        stepsReq = 5; % Steps of minDiff required for convergence
        stepsSS; % Steps taken since minDiff
        Tarray;
        
        % Saltation matrix stuff:
        Px = [];
        Px_u = [];
        Ba = [];
        B = [];
        Bu = [];
        Astance_nom;
        Bstance_nom;
        Tstance_nom;
    end
    
    methods
        
        %  Class constructor.
        function Sim = Simulation(varargin)
          
            Sim.Mod = HopperModel();
            Sim.Con = Controller();
            Sim.Env = Terrain();
           
        end
                
        function Sim = SetEndCond(Sim, value)
            L = length(value);
            if L<1
                error('Invalid input for EndCond');
            end
            
            if value(1) == 1
                Error = ['When setting EndCond to 1,',...
                           'a value for num. steps is also needed',...
                           '\nPlease use Sim.EndCond = [1,nsteps]'];
                if L<2
                    error(Error);
                else
                    if ~isnumeric(value(2)) || value(2)<1
                        error(Error);
                    end
                end
            end
            
            Sim.EndCond = value;
        end
        
   

        function StopButtonCallback(Sim, hObject, eventdata, handles) %#ok<INUSD>
        
            if Sim.StopSim == 0 
                Sim.StopSim = 1;
                Sim.Out.Type = Sim.EndFlag_StoppedByUser;
                Sim.Out.Text = 'Simulation stopped by user';
                set(hObject,'String','Close Window');
            else
                close all
                %delete(Sim.FigWin)
            end

        end 
        
       function PauseButtonCallback(Sim, hObject, eventdata, handles) %#ok<INUSD>

            if Sim.StopSim == 0 && Sim.PauseSim == 0
                
                Sim.PauseSim = 1; 
                set(hObject,'String','Resume');

                while Sim.PauseSim
                    %do nothing
                    pause(0.1)
                end
                
            elseif  Sim.StopSim == 0 && Sim.PauseSim == 1
                Sim.PauseSim = 0;
                set(hObject,'String','Pause');
            end

       end
        
       function DeleteFcnCallback(Sim,hObject, eventdata, handles)%#ok<INUSD>
             
           if Sim.StopSim == 0 
               
                close all
                Sim.StopSim = 1;
                Sim.Out.Type = Sim.EndFlag_WindowClosed;
                Sim.Out.Text = 'Window closed by user';
                
           end
               
       end
    
        function Sim = SetTime(Sim,tstart,tstep,tend)
            if nargin~=4
                error(['Set time expects 3 input arguments',...
                    ' but was provided with ',num2str(nargin)]);
            end
            Sim.tstart = tstart;
            Sim.tstep_normal = tstep;
            Sim.tstep = tstep;
            if isnumeric(tend)
                if tend<=tstart+tstep
                    error('tend is too close to tstart');
                else
                    Sim.tend = tend;
                end
                Sim.infTime = 0;
            else
                if strcmp(tend,'inf')
                    % Simulation will run for indefinite time
                    Sim.infTime = 1;
                    Sim.tend = 10;
                end
            end
        end

        function [Xt] = Derivative(Sim,t,X)

            Xt = [Sim.Mod.Derivative(t,X(Sim.ModCo));
                  Sim.Con.Derivative(t,X(Sim.ConCo))];
             
        end
    
        function [value, isterminal, direction] = Events(Sim, t, X) %#ok<INUSL>
            
            value = zeros(Sim.nEvents,1);
            isterminal = ones(Sim.nEvents,1);
            direction = zeros(Sim.nEvents,1);

            % Call model event function
            [value(Sim.ModEv), isterminal(Sim.ModEv), direction(Sim.ModEv)] = ...
                Sim.Mod.Events(X(Sim.ModCo), Sim.Env);
            % Call controller event function
            [value(Sim.ConEv), isterminal(Sim.ConEv), direction(Sim.ConEv)] = ...
                Sim.Con.Events(X(Sim.ConCo));
            
        end
        
        function [] = RecordEvents(Sim,TE,YE,IE)
            
           Sim.EventsCounter = Sim.EventsCounter+1;

           Sim.Out.EventsVec.Type{Sim.EventsCounter} = IE(end);
           Sim.Out.EventsVec.Time{Sim.EventsCounter} = TE(end);
           Sim.Out.EventsVec.State{Sim.EventsCounter} = YE(end,:);
           
           
        end
        
   
    end
end
