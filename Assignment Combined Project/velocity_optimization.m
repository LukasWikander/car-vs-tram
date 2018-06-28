function res = velocity_optimization(vehicle_params, general_params, dc)
%VELOCITY_OPTIMIZATION Summary of this function goes here
%   Detailed explanation goes here
% Sample code for optimal cruise control of an electric vehicle in a hilly
% terrain. All problem settings are saved in structure task. The vehicle
% parameters are saved in structure task.V. The optimal results are saved
% in structure task.res.
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Created by Nikolce Murgovski, 2017-05.

vref = velocity_profile(general_params, dc);

task=struct;                    % keep all data in one structure
task.Vmax=general_params.v_max_kmh;                   % [km/h] speed limit on max allowed speed
task.Vmin=15;                   % [km/h] speed limit on min allowed speed (avoid putting very low speeds - this mutilates the cost function)
task.V0=task.Vmin;				        % [km/h] initial vehicle speed
task.printprogress=true;        % print the optimization progress 

% design parameters
task.mass=vehicle_params.m_kg;  % [kg] vehicle mass
task.cd=vehicle_params.Cd;                      % [-] air drag coefficent
task.Af=vehicle_params.Ad;                   % [m^2] front area
task.cr=vehicle_params.Cr;                 % [-] rolling resistance coefficient
task.Rw=vehicle_params.rw;                  % [m] wheel radius
task.gearratio=1;               % [-] gear ratio if needed (final gear is 2.8)
task.Pemmax = 200000;		% Maximum torque (for grid purposes)

task = dynprog_settings(task, vehicle_params, general_params, dc);

%% Dynamic Programming
% Gridded states and control signals
X=linspace(task.Vmin/3.6, task.Vmax/3.6,  50)'; % grid on vehicle speed
U=linspace(min(task.V.em.Tmin),max(task.V.em.Tmax), 80)'; % grid on EM torque
Nx=numel(X); Nu=numel(U);
% Memory allocation
costmatrix=1e3*ones(task.N+1,Nx);               % initialize the cost matrix with high cost
costmatrix(task.N+1,:)=[0 1e3*ones(1,Nx-1)];%1e6*(0:Nx-1);             % there is no target cost. Final velocity is not constrained.

tic;
% Optimize backwards in travel distance 
fprintf('%d',task.N)
for tix=task.N:-1:1    
    foundfeasx=false;
	for xix=1:Nx        % loop through gridded state values 
        v=X(xix);
        [vupd,Tm,Pb]=updatestates(task,tix,v,U); % update state
        if ~isempty(vupd)
            foundfeasx = true;
            %instcost=task.ds*(task.energypenalty*Pb/3.6e6 + task.traveltimepenalty/3.6)/v ...
            %    + task.accpenalty*(v-vupd).^2; % consumed el. energy + travel cost + acceleration penalty in [kWh]
            instcost = dynprog_costfcn(v, vupd, Pb, task);
			costmatrix(tix,xix)=min(instcost + interp1(X,costmatrix(tix+1,:)',vupd));   % Bellman's principle of optimality
        end
	end
	
	if task.printprogress
		for j=0:log10(tix+1)
			fprintf('\b'); % delete previous counter display
		end
		fprintf('%d',tix);
	end
	
    if ~foundfeasx
        error('The problem is infeasible at instance %d!',tix);
    end
end
fprintf('\bDone!\n')

% The problem is solved and the optimal policy is within costmatrix. We can
% now choose a desired initial value and obtain the optimal control
% trajectory by simulating forward.
vopt=NaN(task.N+1,1);
Tmopt=NaN(task.N,1);
Pbopt=NaN(task.N,1);
% start from a desired state value
vopt(1)=task.V0/3.6;
for tix=1:task.N
    v=vopt(tix); 
    [vupd,Tm,Pb]=updatestates(task,tix,v,U);
    %instcost=task.ds*(task.energypenalty*Pb/3.6e6 + task.traveltimepenalty/3.6)/v ...
    %    + task.accpenalty*(v-vupd).^2; % consumed el. energy + travel cost + acceleration penalty in [kWh]
    instcost = dynprog_costfcn(v, vupd, Pb, task);
	[~,ix]=min(instcost + interp1(X, costmatrix(tix+1,:)', vupd));  % Bellman's principle of optimality
    Tmopt(tix) = Tm(ix);
    Pbopt(tix) = Pb(ix);
    vopt(tix+1)=vupd(ix);
end
comptime=toc;

%%  Post-treat data 
% Consumed el. energy
vopt=vopt(2:end);

%cost=sum(Pbopt./vopt(1:end-1))*task.ds;
cost=sum(Pbopt./vopt)*task.ds;
t = [0; task.ds*cumsum(1./vopt)];
mins = floor(t(end) / 60);
secs = t(end) - mins * 60;
Deltat = t(end) - task.tmax;
if Deltat > 0
    warning('The trip lasted %1.2fs more than allowed!',Deltat);
else
    fprintf('You have finished the trip %1.2fs earlier than the specified constraint. \n',-Deltat);  
end

Deltaref = t(end) - vref.t(end);
if Deltaref >= 0
	fprintf('Compared to an accelerate-drive-decelerate scheme you are %1.2f seconds slower. \n',Deltaref)
else
	fprintf('Compared to an accelerate-drive-decelerate scheme you are %1.2f seconds faster. \n',-Deltaref)
end

% Battery state of charge
%deltaE_batt_kWh = -task.ds*cumsum(Pbopt./vopt(1:end-1)) /1000 /60/60;
deltaE_batt_kWh = -task.ds*cumsum(Pbopt./vopt) /1000 /60/60;
Ereq_batt_kWh = max(deltaE_batt_kWh) - min(deltaE_batt_kWh);

% Braking torque
Fair = task.V.chs.cd*task.V.chs.Af*task.env.airdensity/2*vopt(1:end-1).^2;    % aerodynamic drag
Fslope = task.V.chs.m*task.env.gravity*sin(task.dc.slope);               % force due to road inclination
Froll = task.V.chs.m*task.env.gravity*task.V.chs.cr*cos(task.dc.slope);       % rolling resistance
%Tbrk = Tmopt - task.V.R*(Fair + Fslope + Froll ...
%    + diff(vopt)/task.ds.*vopt(1:end-1)*task.V.chs.m);

% Print result
fprintf('Solved: battery size needed=%1.2f kWh, travel time=%1.0f.%1.0fmin, computation time=%1.2f s\n\n', ...
    cost/3.6e6,mins,secs,comptime);

% save optimal results in a structure
res.v=vopt;
res.Pbopt=Pbopt;
res.Tm=Tmopt;
%res.Tbrk=Tbrk;
res.deltaE_batt_kWh=deltaE_batt_kWh;
res.Ereq_batt_kWh=cost;
res.t=t;

res.vref=vref;

end

