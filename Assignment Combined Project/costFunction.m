function y = costFunction(x,tram_params, car_params, general_params,...
    drv_mission, pass_flow, tram_full_sim_output, car_full_sim_output,...
    tram_empty_sim_output,car_empty_sim_output)

% Initialize for the objective 
y(1) = 0;
y(2) = 0;

%Fleet size estimation
tram_params.E_battery_size_kWh = x(1);
car_params.E_battery_size_kWh = x(2);
fleet_info_output = fleet_size_estimation(tram_params, car_params, general_params, drv_mission, pass_flow, general_params.n_variations);

% Cost estimation
% TODO: Use empty vehicle simulation to get more accurate costs
cost_estimation_output = cost_estimation(tram_params, car_params, general_params, tram_full_sim_output, car_full_sim_output, fleet_info_output, false);

% Output
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
ROI_yr = general_params.ROI_horizon_yr; % Time at which to evaluate cost [year]

opt = assignment_cost_function(outputs, ROI_yr);

% Compute the cost function
y(1) = opt.cost;
y(2) = opt.energConsum;
end