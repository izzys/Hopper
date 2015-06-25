    % Real time plot for the whole system
    function status = RealTimePlot(Sim,t,X,flag)
        if strcmp(flag,'done')
            % Finish simulation
            return
        end
               
        if Sim.Graphics
            if strcmp(flag,'init')
                if Sim.Once
                    % Initialize
                    scrsz = get(0, 'ScreenSize');
                    Sim.FigWin=figure();
                    
                    set(Sim.FigWin,'DeleteFcn' ,@Sim.DeleteFcnCallback)
                    set(Sim.FigWin,'Position', [scrsz(3)*Sim.WindowLeftLocation scrsz(4)*0.15 scrsz(3)*0.5 scrsz(4)*0.7]);
                    set(gca,'LooseInset',get(gca,'TightInset')*2)

                    hold on
                    axis([Sim.FlMin Sim.FlMax Sim.HeightMin Sim.HeightMax]);
                    axis equal

                    % Draw Robot
                    Sim.Mod=Sim.Mod.Render(X(Sim.ModCo));
                    % Draw Floor
                    Sim.Env=Sim.Env.Render(Sim.FlMin,Sim.FlMax);

                    % Display time
                    Sim.TimeDispObj = text(1.32, 1.6, sprintf('t=%.2f',t(1)),'HorizontalAlignment','center');%,'Parent',tCOM);

                    Sim.StopButtonObj = uicontrol('Style', 'pushbutton', 'String', 'Stop Simulation',...
                         'Units','normalized',...
                         'Position', [ 0.86 0.9 0.09 0.04],...
                         'Callback', @Sim.StopButtonCallback);
                     
                    Sim.PauseButtonObj = uicontrol('Style', 'pushbutton', 'String', 'Pause ',...
                         'Units','normalized',...
                         'Position', [ 0.86 0.95 0.09 0.04],...
                         'Callback', @Sim.PauseButtonCallback);
                    
                    Sim.Once=0;
                end
                
                return
            end

                
            % Draw Robot
            Sim.Mod.Render(X(Sim.ModCo));
            % Draw Floor
            Sim.Env.Render(Sim.FlMin,Sim.FlMax);

            set(Sim.TimeDispObj,'String',sprintf('t=%.2f',t(1)));
            
            drawnow
            
        end
        
        status=Sim.StopSim;
    end