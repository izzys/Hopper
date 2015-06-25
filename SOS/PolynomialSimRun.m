function Flag = PolynomialSimRun(F,scale,x0)

k_max = 50;
x0_norm = 1;
k = 1;
Flag = 3;%not converge:
xk = x0;
pvar x1 x2

while k<k_max && x0_norm<100
    
xk = subs(F,[x1;x2],xk);
 
x0_norm = norm(double(xk));

    if x0_norm<1e-5
        %coverge:
        Flag =    2;
        break
    end

k=k+1;
end






