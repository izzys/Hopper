function [EigVal,EigVec] = Poincare( Sim )
% Calculates the Linearized Poincare eigenvalues
disp 'Calculating Poincare section eigen-values...'

Ncoord = length(Sim.ModCo);
Coords = 1:Ncoord;

% Limit cycle initial conditions
IC = repmat([Sim.LC 0]', 1, Ncoord);%IC = repmat(Sim.IClimCyc, 1, Ncoord);

% Disturbed initial conditions
dIC = IC;
dICp = IC;
for d = Coords
    dIC(d,d) = dIC(d,d) + Sim.PMeps;
end

% Run the simulations
PMsim = copy(Sim);
PMsim.EndCond = [1,Sim.Period(1)];

for d = Coords
    
    disp(['Running simulation for coordinate #', num2str(d)])
    PMSim.LC = Sim.LC;
    PMsim = PMsim.Set('Graphics',0);
    PMsim.IC = dIC(:,d)';
    PMsim = PMsim.Init();
    PMsim = PMsim.Run();

    if PMsim.Out.Type ~= 1
        % Robot didn't complete the step
        EigVal = 2*ones(Ncoord,1);
        EigVec = eye(Ncoord);
        disp 'Robot didn''''t complete the step'
        return;
    end

    dICp(:,d) = PMsim.ICstore(:,1);
end

% Calculate deviation

% Analytic:
e =eig(Sim.Px);
disp 'Px Eigen-values:'
disp(e)

e2 =eig(Sim.Px-Sim.Bu*Sim.Con.Kcontroller);
disp 'Px-Bu*K Eigen-values:'
disp(e2)

%Numeric:
DP = 1/Sim.PMeps*(dICp(Coords,:) - IC(Coords,:));
[EigVec,EigVal] = eig(DP,'nobalance');
EigVal = diag(EigVal);
end