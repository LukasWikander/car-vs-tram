function [PtoTem,EtaEM,Tem,Wem,Tem_lim,EnergyConsEM] = CreateEMmap(Pem_max,wem_max,Tem_max)

clear i j Psi Isx Isy IsyMAX Curr Tmax T Pout Pcopper Piron Pwindage Ploss Eta
fwr=wem_max/(Pem_max/(Tem_max));  % field weakening ratio
idyn=1; % max current / continuous current
Demag=1/0.7; % it takes 1/Demag times the nominal current to demagnetize the machine

w=[0:fwr/20:fwr];
Tref=[-idyn:idyn/25:idyn];

for i=1:length(Tref)
    for j=1:length(w)
        Psi(i,j)=min(1,1/(eps+w(j)));
        Isx(i,j)=min(idyn,max((w(j)-1),0)/(max(w)-1)/Demag);
        Isy(i,j)=min(abs(Tref(i))/Psi(i,j),sqrt(idyn^2-Isx(i,j)^2));
        IsyMAX(i,j)=min(max(Tref)/Psi(i,j),sqrt(idyn^2-Isx(i,j)^2));
        Curr(i,j)=sqrt(Isx(i,j)^2+Isy(i,j)^2);
        Tmax(i,j)=min(Tem_max,Pem_max/(eps+w(j)));  % Pem_maxPsi(i,j)*IsyMAX(i,j);
 
        T(i,j)=min(Tref(i),Tmax(i,j));
        Pout(i,j)=w(j)*T(i,j);
        Pcopper(i,j)=(Curr(i,j)/idyn)^2*0.02;
        Piron(i,j) = w(j)*Psi(i,j)^2*0.005 + (w(j)*Psi(i,j))^2*0.01;
        Pwindage(i,j)=(w(j)/max(w))^3*0.005;
        Ploss(i,j)=0.005+Pcopper(i,j)+Piron(i,j)+Pwindage(i,j);
        Eta(i,j)=min(1,(max(0.02,(abs(Pout(i,j))/(eps+abs(Pout(i,j))+Ploss(i,j)))))); 
    end
end

Tem = Tem_max.*Tref/max(Tref);
Wem = wem_max.*w/max(w);
Pem = [-Pem_max:Pem_max/50:Pem_max];

Tem_lim = min([Tem_max*ones(1,length(Wem)); Pem_max./Wem]);

EtaEM=Eta;

for i=1:length(Tem)
    for j=1:length(Wem)
        if Wem(j)*Tem(i)>0
            EnergyConsEM(i,j)=min(Pem_max,Wem(j)*Tem(i))+Ploss(i,j)*Pem_max;
        else
            EnergyConsEM(i,j)=max(-Pem_max,Wem(j)*Tem(i))+Ploss(i,j)*Pem_max;
        end
    end
end
% EnergyConsEM(1,:)=Wem(:)'./EtaEM(2,:)*Tem(2);

PtoTem = zeros(length(Pem),4);
PtoTem(:,1) = Pem;

for i=1:length(Pem)
    for j=2:length(Tem)
        Ttemp=Tem(j);
        wtemp = (Pem(i) / Ttemp);
        if abs(wtemp) > wem_max 
            wtemp = wem_max;
            eta(j)=0;
        else
            eta(j) = interp2(Wem,Tem,EtaEM,wtemp,Ttemp,'linear');
        end
        if eta(j) > PtoTem((i),3)
            PtoTem((i),2) = Tem(j);
            PtoTem((i),3) = eta(j);
            PtoTem((i),4) = wtemp;
        end
    end
end

for i=2:length(Pem)-1 % Smooth a bit to make simulation less jumpy
    PtoTem(i,2)=(PtoTem(i-1,2)+PtoTem(i,2)+PtoTem(i+1,2))/3;
    PtoTem(i,4)=(PtoTem(i-1,4)+PtoTem(i,4)+PtoTem(i+1,4))/3;
end

%{
figure(2)
set(gcf,'OuterPosition',[10,360,1300,350])
clf

subplot(1,4,1)
surfc(Wem,Tem,EtaEM)
%axis([0 max(Tem) -max(Wem)*0 max(Wem) 0 1])
xlabel('Speed [rad/s]')
ylabel('Torque [Nm]')
title('Electrical machine efficiency')

subplot(1,4,2)
surfc(Wem,Tem,EnergyConsEM)
%axis([0 max(Wem) 0 max(Tem) 0 1])
xlabel('Speed [rad/s]')
ylabel('Torque [Nm]')
title('Electrical machine Energy Consumption')

subplot(1,4,3)
plot(PtoTem(:,4),PtoTem(:,2),'b*-')
hold on
plot(Wem,Tem_lim,'r');
hold on
plot(Wem,-Tem_lim,'r');
%axis([0 wem_max 0 Tem_max*1.1])
title('Optimal and maximum torque [Nm]')
xlabel('Speed [rad/s]')
ylabel('Torque [Nm]')
grid on

subplot(1,4,4)
plot(PtoTem(:,1),PtoTem(:,3),'b')
hold on
%axis([0 Pem_max 0 1])
title('Optimal efficiency')
xlabel('Power [W]')
ylabel('Efficiency')
grid on
%}
