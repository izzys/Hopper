function []= PlotResults(Sim)




figure(11)

subplot 221

plot(Sim.Out.T,Sim.Out.X(:,1))
hold on
xlabel('Time [sec]')
ylabel('x1 [m]')
plot_event(1)
grid on

subplot 222

plot(Sim.Out.T,Sim.Out.X(:,2))
hold on
xlabel('Time [sec]')
ylabel('x2 [m]')
plot_event(2)
grid on

subplot 223
plot(Sim.Out.T,Sim.Out.X(:,3))
hold on
xlabel('Time [sec]')
ylabel('x1 dot [m]')
plot_event(3)
grid on

subplot 224
plot(Sim.Out.T,Sim.Out.X(:,4))
hold on
xlabel('Time [sec]')
ylabel('x2 dot [m]')
plot_event(4)
grid on

figure(12)

subplot 211
for i=1:size(Sim.Out.X,1)
l(i) = (Sim.Mod.GetSpringLength(Sim.Out.X(i,Sim.ModCo))-Sim.Mod.spr_l0);
Nf(i) = Sim.Mod.spr_k*l(i)+Sim.Mod.damping*Sim.Out.X(i,4)-Sim.Mod.m1*Sim.Mod.g;
end
plot(Sim.Out.T,Nf)
hold on
xlabel('Time [sec]')
ylabel('normal forcr [N]')
line([0 Sim.Out.T(end)],[ 0 0],'LineStyle','--','Color','r')
plot_event(0)

subplot 212
plot(Sim.Out.T,l)
hold on
xlabel('Time [sec]')
ylabel('Spring deformation [m]')
line([0 Sim.Out.T(end)],[ 0 0],'LineStyle','--','Color','r')
plot_event(0)

figure()
NS = 0:1:Sim.StepsTaken;

plot(NS,Sim.Out.delta_actual(4,:),'b*')
hold on
%plot(NS,Sim.Out.delta_linear(4,:),'mo')
hold on
xlabel('# steps')
ylabel('\delta x2 dot [m]')
grid on
legend('Actual error','Linear Error (By saltation matrix)')


    function [] = plot_event(state)
        
        if ~isempty(Sim.Out.EventsVec)
        
        for j = 1:length(Sim.Out.EventsVec.Type)
            
            switch Sim.Out.EventsVec.Type{j}
                
                case 1
                    
                    if state
                    plot(Sim.Out.EventsVec.Time{j},Sim.Out.EventsVec.State{j}(state), 'r*','MarkerSize',6,'LineWidth',2)
                    else
                    plot(Sim.Out.EventsVec.Time{j},0, 'r*','MarkerSize',6,'LineWidth',2)   
                    end
                    
                    
                case 2
                    
                    if state
                    plot(Sim.Out.EventsVec.Time{j},Sim.Out.EventsVec.State{j}(state), 'go','MarkerSize',6,'LineWidth',2)
                    else
                    plot(Sim.Out.EventsVec.Time{j},0, 'go','MarkerSize',6,'LineWidth',2)   
                    end
                    
            end
                    
        end
     
        end
    
    end

    plot_ellipse(Sim.Mod,Sim.Out.X)

end

