%EOA check:
clear all;close all;clc
load('PolyApproxByEric.mat')
load('RoAestimatedByEric.mat') 
Sim = HopperSimInit() ;

pvar x1 x2
figure(332)
[C,~] = pcontour(V,-gamma,[-0.07 0.07 -0.5  0.35]);
hold on
grid on
title('RoA estimation')
[kk_x1,kk_x2]=size(C);
for k_to_scale_C = 0.7:0.1:1.5
k_to_scale_C
Ck = k_to_scale_C*C;

    for kk =2:5:kk_x2
    x0 = Ck(:,kk);

    % Numeric simulation:
    NumericFlag = HopperSimRun(Sim,x0) ;
    mark_collor = MarkForFlag(Sim,NumericFlag);
    figure(332)
    plot(x0(1),x0(2),mark_collor,'Marker','o','MarkerSize',8,'LineWidth',2)


    % Polynomial check:
    PolyFlag = PolynomialSimRun(F,scale,x0) ;
    mark_collor = MarkForFlag(Sim,PolyFlag);
    figure(332)
    plot(x0(1),x0(2),mark_collor,'Marker','x','MarkerSize',6)

    end


end

disp 'done'