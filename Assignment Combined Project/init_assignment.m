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

%% Fleet size estimation
tram_params.t_round_trip = tram_full_sim_output.t_round_trip;
tram_params.t_charging_round_trip = tram_full_sim_output.t_charging_round_trip;
tram_params.E_battery_size_kWh = 4*2*tram_full_sim_output.req_battery_size_kWh;
tram_params.E_round_trip_kWh = tram_full_sim_output.E_tot_kWh;
car_params.t_round_trip = car_full_sim_output.t_round_trip;
car_params.t_charging_round_trip = car_full_sim_output.t_charging_round_trip;
car_params.E_battery_size_kWh = 1.75*2*car_full_sim_output.req_battery_size_kWh;
car_params.E_round_trip_kWh = car_full_sim_output.E_tot_kWh;
fleet_info_output = fleet_size_estimation(tram_params, car_params, general_params, drv_mission, pass_flow, general_params.n_variations);

%% Cost estimation
% TODO: Use empty vehicle simulation to get more accurate costs
cost_estimation_output = cost_estimation(tram_params, car_params, general_params, tram_full_sim_output, car_full_sim_output, fleet_info_output);


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

%% Output
outputs.params.tram = tram_params;
outputs.params.car = car_params;
outputs.params.general = general_params;
outputs.params.drv_mission = drv_mission;
outputs.params.pass_flow = pass_flow;
outputs.results.tram_full = tram_full_sim_output;
outputs.results.tram_empty = tram_empty_sim_output;
outputs.results.car_full = car_full_sim_output;
outputs.results.car_empty = car_empty_sim_output;
outputs.results.fleet = fleet_info_output;
outputs.results.costs = cost_estimation_output;

end
