function [V,bet,gamma] = discrete_sys_ROA(f,x,V0,p0,opts)
% f - discrete map, x - vector of independent variables of f, V0 - initial
% guess for lyapunov function, p - shape factor, Niter - Number of
% iterations, opts - sosopts for SOS optimization
    pvar t beta;
    V = V0;
    bet = 0;
    k = 0;
    p = p0;
    d = 1;
    count=1;
    if isfield(opts,'FileName')
        FileName = opts.FileName;
    else
        FileName = 'ROA_opt';
    end
    
    if isfield(opts,'sos_opts')
        sos_opts = opts.sos_opts;
    else
        sos_opts = gsosoptions;
        %opts.minobj = -0.0001; 
        sos_opts.minobj = -1;
        sos_opts.maxobj = 0;
        sos_opts.relbistol = 1e-7;
        sos_opts.absbistol = 1e-7;
        sos_opts.display = 'on';
        sos_opts.scaling = 'on';
    end
    
    if isfield(opts,'reltol')
        reltol = opts.reltol;
    else
        reltol = 0.01;
    end
    
    if isfield(opts,'MaxIter')
        Niter = opts.MaxIter;
    else
        Niter = 50;
    end
    
    if isfield(opts,'SFadapt')
        SFadapt = opts.SFadapt;
    else
        SFadapt = 1;
    end

    if isfield(opts,'DegV')
        DegV = opts.DegV;
    else
        DegV = 2;
    end
    
    if isfield(opts,'N2')
        N2 = opts.N2;
    else
%         N2 = f.maxdeg + 1;
          N2 = 2*DegV - p.maxdeg;
    end
    
    if isfield(opts,'N1')
        N1 = opts.N1;
    else
%         N1 = f.maxdeg + 1;
          N1 = f.maxdeg*2*DegV - 2*DegV;
    end
    

    
    
    while abs(d) > reltol && k<Niter %(bet - b_old) > 1e-3
        k = k+1;
        b_old = bet;
        % varaiable declaration
        s1 = sosdecvar('a',monomials(x,0:N1/2));
        %
        DV = subs(V,x,f)-V;
        % Maximize alpha
        sosc = polyconstr;
        sosc(1) = s1>=0;
        sosc(2) = -(1e-4*(x.'*x) - s1*(V+t)+DV) >= 0;
        [info,dopt,sossol] = gsosopt(sosc,x,t,sos_opts);
        %Maxsimize beta
        s1 = subs(s1,dopt);
        gamma = info.tbnds(2);
        s2 = sosdecvar('b',monomials(x,0:N2/2));
        sosc2 = polyconstr;
        sosc2(1) = s2>=0;
        sosc2(2) = -((V+gamma)-(t+p)*s2)>=0;
        [info2,dopt2,sossol2] = gsosopt(sosc2,x,t,sos_opts);
        s2 = subs(s2,dopt2);
        if ~SFadapt
           %optimize V  constant shape factor [Z.Jarvis]
            V_new = sosdecvar('c',monomials(x,1:DegV));
            DV = subs(V_new,x,f)-V_new;
            sosc3 = polyconstr;
            sosc3(1) = -(1e-4*(x.'*x) - s1*(V_new+gamma)+DV) >= 0;
            sosc3(2) = -((V_new+gamma)-(t+p)*s2)>=0;
            sosc3(3) = V_new - 1e-4*(x.'*x)>=0;
            [info3,dopt3,sossol3] = sosopt(sosc3,x,t,opts);
            V = subs(V_new,dopt3);
            bet = -double(subs(t,dopt3));
            d = (bet - b_old)/bet;
        else
            %optimize V with adaptive shape factor [L. Khodadadi] 
            bet = -info2.tbnds(2);
            V_new = sosdecvar('c',monomials(x,1:DegV));
            DV = subs(V_new,x,f)-V_new;
            sosc3 = polyconstr;
            sosc3(1) = -(1e-4*(x.'*x) - s1*(V_new+gamma)+DV) >= 0;
            sosc3(2) = -((V_new+gamma)-(-bet+p)*s2)>=0;
            sosc3(3) = V_new - 1e-4*(x.'*x)>=0;
            [info3,dopt3,sossol3] = sosopt(sosc3,x,sos_opts);
            p = cleanpoly(V,[],[0:2]);
            V = subs(V_new,dopt3);
            d = (bet - b_old)/bet;
        end
        disp(['diff:',num2str(d),' k:',num2str(k)]);
        save([FileName,datestr(count),'.mat'],'V','bet','gamma','s1','s2','p','b_old')

    end
end
