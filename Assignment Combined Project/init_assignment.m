function outputs = init_assignment(general_params,tram_params,car_params,doPlot)

%% Load variables, initialise
drv_mission  = load ('dc_AtoB.mat');
pass_flow    = load ('passengers.mat');

l = diff(drv_mission.dc.s);
M = [0.5*eye(length(drv_mission.dc.slope)-1) zeros(length(drv_mission.dc.slope)-1,1)] ...
	+ [zeros(length(drv_mission.dc.slope)-1,1) 0.5*eye(length(drv_mission.dc.slope)-1)]; % Mean adjacent value matrix

a = M*drv_mission.dc.slope; 
drv_mission.dc.pos = cumsum([0; l.*cos(a)]); % Flight distance [m]
drv_mission.dc.alt = cumsum([drv_mission.dc.altinit; l.*sin(a)]);

general_params.l_trip = drv_mission.dc.s(end); % Length of trip [m]
general_params.l_round_trip = 2*drv_mission.dc.s(end); % Length of round trip [m]

%% Vehicle simulation
% Full tram
fprintf(' --- Full tram simulation --- \n')
tram_full_sim_output = vehicle_simulation(tram_params, general_params, drv_mission, pass_flow);

% Empty tram
fprintf(' --- Empty tram simulation --- \n')
tram_empty_params = tram_params;
tram_empty_params.name = 'tram_empty';
tram_empty_params.n_pass = 0;
tram_empty_sim_output = vehicle_simulation(tram_empty_params, general_params, drv_mission, pass_flow);

% Full car
fprintf(' --- Full car simulation --- \n')
car_full_sim_output = vehicle_simulation(car_params, general_params, drv_mission, pass_flow);

% Empty car
fprintf(' --- Empty car simulation --- \n')
car_empty_params = car_params;
car_empty_params.name = 'car_empty';
car_empty_params.n_pass = 0;
car_empty_sim_output = vehicle_simulation(car_empty_params, general_params, drv_mission, pass_flow);

%% Fleet size parameters
tram_params.t_round_trip = tram_full_sim_output.t_round_trip;
tram_params.t_charging_round_trip = tram_full_sim_output.t_charging_round_trip;
tram_params.E_round_trip_kWh = tram_full_sim_output.E_tot_kWh;
car_params.t_round_trip = car_full_sim_output.t_round_trip;
car_params.t_charging_round_trip = car_full_sim_output.t_charging_round_trip;
car_params.E_round_trip_kWh = car_full_sim_output.E_tot_kWh;

%% Run optimization for the battery sizes
if doPlot
    costFun = @(x)costFunction(x,tram_params, car_params, general_params,...
        drv_mission, pass_flow, tram_full_sim_output, car_full_sim_output,...
        tram_empty_sim_output,car_empty_sim_output); % Cost function of the optimization problem
    nVars = 2;      % Number of decision variables
    A = [];         % Linear inequalities: A*x <= b
    b = [];         % Linear inequalities: A*x <= b
    Aeq = [];       % Linear inequalities: Aeq*x = beq
    beq = [];       % Linear inequalities: Aeq*x = beq
    lb = [2*4*tram_full_sim_output.req_battery_size_kWh; 2*4*car_full_sim_output.req_battery_size_kWh]; % Lower boundaries: lb <= x <= ub
    ub = [];        % Upper boundaries: lb <= x <= ub
    nonlcon = [];   % Function nonlcon(x): returns C and Ceq and ga minimizes costFun so that C(x) <= 0 and Ceq(x) = 0                             
    optionsGA = optimoptions('ga','ConstraintTolerance',1e-6,...
        'MaxGenerations',200,'PopulationSize',50,'PlotFcn', @gaplotbestf);
    optionsGAMO = optimoptions('gamultiobj','ConstraintTolerance',1e-6,...
        'MaxGenerations',5,'PopulationSize',50,'PlotFcn', @gaplotpareto);
    [x,fval,exitflag,output] = gamultiobj(costFun,nVars,A,b,Aeq,beq,lb,ub,nonlcon,optionsGAMO);
    save('OptimizationData','x','fval');
else
    load('OptimizationData','x','fval');
end

%Finding the cases with minimum total cost and minimum energy consumption
[~, indexes] = min(fval,[],1);

%Selecting the case with minimum total cost for plotting
tram_params.E_battery_size_kWh = x(indexes(1),1);
car_params.E_battery_size_kWh = x(indexes(1),2);

%% Calculation for the selected values derived from the optimization
fleet_info_output = fleet_size_estimation(tram_params, car_params, general_params, drv_mission, pass_flow, general_params.n_variations);
cost_estimation_output = cost_estimation(tram_params, car_params, general_params, tram_full_sim_output, car_full_sim_output, fleet_info_output);
outputs.results.fleet = fleet_info_output;
outputs.results.costs = cost_estimation_output;
ROI_yr = general_params.ROI_horizon_yr; % Time at which to evaluate cost [year]
opt = assignment_cost_function(outputs, ROI_yr);

%% Warn if unable to manage with selected battery sizes
if any(fleet_info_output.fleet_constraint_compliance == 0)
	n_noncompliant_cars = sum(fleet_info_output.car_constraint_compliance == 0);
	n_noncompliant_trams = sum(fleet_info_output.tram_constraint_compliance == 0);
	if n_noncompliant_cars
		throw(MException('INIT_ASSIGNMENT:InfeasibleCarBattery',...
			['Infeasible with selected car battery size for ' num2str(n_noncompliant_cars) ' configurations.']));
	end
	if n_noncompliant_trams
		throw(MException('INIT_ASSIGNMENT:InfeasibleTramBattery',...
			['Infeasible with selected tram battery size for ' num2str(n_noncompliant_trams) ' configurations.']));
	end
	
end

%% Plots
% TODO: Plots
if doPlot
	plot_assignment(tram_params, car_params, general_params, drv_mission, pass_flow, tram_full_sim_output, car_full_sim_output, fleet_info_output, cost_estimation_output);
end

fprintf('Done!\n');

end
