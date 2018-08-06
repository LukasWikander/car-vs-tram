function [Eta,Ptrm,Mbatt,Pbatt_max_new,Wbatt_new]=CreateBATTmap(Pbatt_max,Wbatt)

% Based on Kokam batteries data of Large Capacity LiIon cells as of February 2010
% With Power/energy ratios above 5 W/Wh Power Optimized batteries are
% assumed with 120 Wh/kg and 450 W/kg. Otherwise Energy Optimized cells
% are assumed with 150 Wh/kg and 150 W/kg. The maximum output power is
% about the double in any case.

% assume emf - resistance model with fix resistance and emf
% assume generatoric reference, i.e. positive power discharges the battery
% Ploss = Ri^2
% Pterm_max = (e-R*e/2/R)*e/2/R (Power adaption) = e^2*(1/2/R-1/4R)=e^2*1/4/R;
% R=e^2/(4*Pterm_max)

if Pbatt_max/(Wbatt/3600)>5
    % Power optimized battery
    Mbatt=max(Pbatt_max/450,Wbatt/(120*3600));
    Type = 'Power Optimized';
    Pbatt_max_new = 450*Mbatt;
    Wbatt_new = 120*3600*Mbatt;
else
    % Energy Optimized
    Mbatt=max(Pbatt_max/150,Wbatt/(150*3600));
    Type = 'Energy Optimized';
    Pbatt_max_new = 150*Mbatt;
    Wbatt_new = 150*3600*Mbatt;
end

Ptrm = [-Pbatt_max:2*Pbatt_max/21:Pbatt_max]'; % Terminal power

Pdouble=2*Pbatt_max;  % assume that max power is discharged at double rated power
e=200;
R=e^2/4/Pdouble/2;

for i=1:length(Ptrm)
    curr(i)=(-e/2/R + sqrt(e^2/4/R^2+Ptrm(i)/R));
    volt(i)=e+R*curr(i);
    Ploss(i)=R*curr(i)^2;
    Eta(i)= (Ptrm(i)-Ploss(i))/(Ptrm(i)+eps);
end

%{
figure(3)
clf
plot(Ptrm,Eta)
axis([-Pbatt_max Pbatt_max 0 2])
title('Battery charge efficiency')
xlabel('Battery power [W]')
ylabel('Efficiency')
grid on

text(-Pbatt_max/2,1.9,['Pmax ' num2str(Pbatt_max/1000) '/' num2str(Pbatt_max_new/1000) '  [kW]'])
text(-Pbatt_max/2,1.7,['Wbatt ' num2str(Wbatt/3600/1000) '/' num2str(Wbatt_new/3600/1000) '  [kWh]'])
text(-Pbatt_max/2,1.5,[Type '  cells'])
text(-Pbatt_max/2,1.3,[num2str(Mbatt) '  [kg]  ' num2str(Pbatt_max_new/Mbatt/1000) ' [kW/kg]   ' num2str(Wbatt_new/3.6e6/Mbatt) ' [kWh/kg]'])
%}
