function [ output ] = cost_estimation( tram_params, car_params, general_params, tram_simulation_output, car_simulation_output, fleet_info_output )
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here

output = struct;

fprintf('\n--Tram round trip drive cycle cost calculation--\n');
tram_drive_cycle_cost = drive_cycle_cost(tram_params, general_params, tram_simulation_output);
output.tram_drive_cycle_cost = tram_drive_cycle_cost;

fprintf('\n--Car round trip drive cycle cost calculation--\n');
car_drive_cycle_cost = drive_cycle_cost(car_params, general_params, car_simulation_output);
output.car_drive_cycle_cost = car_drive_cycle_cost;

fprintf('\n--Operational day cost--\n');
%tram_fleet_cost_grid = fleet_info_output.tram_freq * (tram_drive_cycle_cost.c_E_round_trip + tram_drive_cycle_cost.c_M_round_trip);
tram_fleet_cost_grid = fleet_info_output.tram_energy_charged_grid * general_params.c_E_kWh + ...
	(fleet_info_output.tram_freq_A2B + fleet_info_output.tram_freq_B2A) * tram_drive_cycle_cost.c_M_round_trip;

total_car_trips_1hr = (fleet_info_output.car_trips_A2B + fleet_info_output.car_trips_B2A + ...
	fleet_info_output.car_empty_trips_A2B + fleet_info_output.car_empty_trips_B2A);
%car_fleet_cost_grid = fleet_info_output.car_freq * (car_drive_cycle_cost.c_E_round_trip + car_drive_cycle_cost.c_M_round_trip);
car_fleet_cost_grid = fleet_info_output.car_energy_charged_grid * general_params.c_E_kWh + ...
	total_car_trips_1hr * car_drive_cycle_cost.c_M_round_trip;

fleet_cost_grid = tram_fleet_cost_grid + car_fleet_cost_grid;
output.tram_fleet_cost_grid = tram_fleet_cost_grid;
output.car_fleet_cost_grid = car_fleet_cost_grid;
output.fleet_cost_grid = fleet_cost_grid;

% Sum hour costs
fleet_cost_dy = sum(fleet_cost_grid, 2);
output.fleet_cost_dy = fleet_cost_dy;
[min_cost_dy, idx] = min(fleet_cost_dy);
output.min_cost_dy = min_cost_dy;
output.min_operational_alt = idx;

fprintf('Minimum daily operational cost (Not considering equipment costs): \n %1.2f SEK \n', min_cost_dy);
fprintf('Number of trams: %1.0f, number of cars: %1.0f \n', fleet_info_output.num_trams(idx), fleet_info_output.num_cars(idx));


fprintf('\n--Equipment purchase cost--\n');
% Sum equipment cost
% TODO: Costs of parking space, tracks, roads

%% Vehicle purchase cost
cars_cost_purchase = fleet_info_output.num_cars * ...
	(car_params.c_purchase + car_drive_cycle_cost.c_motor + car_drive_cycle_cost.c_batt);
trams_cost_purchase = fleet_info_output.num_trams * ...
	(tram_params.c_purchase + tram_drive_cycle_cost.c_motor + tram_drive_cycle_cost.c_batt);

%% Charging station purchase cost
car_chargers_cost_purchase = fleet_info_output.P_car_chargers ...
    * general_params.c_station_kW;
tram_chargers_cost_purchase = fleet_info_output.P_tram_chargers ...
    * general_params.c_station_kW;

%% Road / track costs
track_cost_purchase = general_params.c_tram_track_km * general_params.l_round_trip * max(fleet_info_output.num_trams, 1);
road_cost_purchase = general_params.c_car_road_km * general_params.l_round_trip * max(fleet_info_output.num_cars, 1);

%% Fleet purchase cost
fleet_cost_purchase =  trams_cost_purchase + cars_cost_purchase ...
    + car_chargers_cost_purchase + tram_chargers_cost_purchase ...
	+ track_cost_purchase + road_cost_purchase;

%%
output.cars_cost_purchase = cars_cost_purchase;
output.trams_cost_purchase = trams_cost_purchase;
output.car_chargers_cost_purchase = car_chargers_cost_purchase;
output.tram_chargers_cost_purchase = tram_chargers_cost_purchase;
output.track_cost_purchase = track_cost_purchase;
output.road_cost_purchase = road_cost_purchase;
output.fleet_cost_purchase = fleet_cost_purchase';
[min_cost_purchase, idx] = min(fleet_cost_purchase);
output.min_cost_purchase = min_cost_purchase;
output.min_upfront_alt = idx;

fprintf('Minimum upfront purchase cost: \n %1.2f SEK \n', min_cost_purchase);
fprintf('Number of trams: %1.0f, number of cars: %1.0f \n', fleet_info_output.num_trams(idx), fleet_info_output.num_cars(idx));


%% Life cycle costs
%distance_driven_per_tram = sum(fleet_info_output.tram_freq, 2) * general_params.l_round_trip /1000 ./ fleet_info_output.num_trams';
distance_driven_per_car_and_day = sum(total_car_trips_1hr, 2) * general_params.l_round_trip /1000 ./ fleet_info_output.num_cars';
output.distance_driven_per_car_and_day = distance_driven_per_car_and_day;
output.l_life_car_yr = general_params.l_life_car_km ./ distance_driven_per_car_and_day /365;


dy = 0:365*80;
daily_cost_grid = output.fleet_cost_dy * dy;
dy_size = size(dy);
n_size = size(output.fleet_cost_dy);

tram_purchase_grid = ceil(ones(n_size) * dy / 365 ./ ((general_params.l_life_tram_yr * ones(n_size)) * ones(dy_size)));
car_purchase_grid = ceil(ones(n_size) * dy /365 ./ (output.l_life_car_yr * ones(dy_size)));
car_purchase_grid(isnan(car_purchase_grid)) = 0; % When using only trams, no cars are purchased

output.cost_grid_mix_day = ((output.trams_cost_purchase' * ones(dy_size)) .* tram_purchase_grid ...
	+ (output.cars_cost_purchase' * ones(dy_size)) .* car_purchase_grid ...
	+ daily_cost_grid) + tram_chargers_cost_purchase' + car_chargers_cost_purchase';



end

function [ output ] = drive_cycle_cost(vehicle_params, general_params, vehicle_simulation_output)
%% Cost of motor
fprintf('Maximum power used during the round trip driving cycle: \n Traction=%1.2f kW, Braking=%1.2f kW \n', ...
	vehicle_simulation_output.P_traction_max_kW,vehicle_simulation_output.P_brake_max_kW);
c_motor = vehicle_simulation_output.P_max_kW * general_params.c_motor_kW;
output.c_motor = c_motor;
fprintf('Cost of motor and power electronics to supply required traction and regenerative braking: \n %1.2f SEK \n',c_motor);
fprintf('Energy used during the round trip driving cycle: \n Traction=%1.2f kWh, Braking=%1.2f kWh \n', ...
    vehicle_simulation_output.E_traction_tot_kWh,vehicle_simulation_output.E_brake_tot_kWh);

%% Cost of battery
EBatt_kWh = 2*vehicle_simulation_output.req_battery_size_energy_kWh;
c_batt = general_params.c_batt_kWh*vehicle_params.E_battery_size_kWh;
output.c_batt = c_batt;
mBatt_kg = vehicle_simulation_output.req_battery_size_kWh/general_params.e_batt_kWh;
tCharge = vehicle_simulation_output.t_charging_round_trip;
fprintf('Battery parameters: \n Required capacity=%1.2f kWh, Actual capacity=%1.2f kWh, Weight=%1.2f kg, Cost=%1.2f SEK, Charge time=%1.2f min \n', ...
    EBatt_kWh,vehicle_params.E_battery_size_kWh,mBatt_kg,c_batt,tCharge/60);

%% Cost of energy used
c_E_round_trip = (vehicle_simulation_output.E_traction_tot_kWh-vehicle_simulation_output.E_brake_tot_kWh)*general_params.c_E_kWh;
output.c_E_round_trip = c_E_round_trip;
fprintf('Cost of energy used: \n %1.2f SEK \n',c_E_round_trip);

%% Cost of maintenance
c_M_round_trip = vehicle_params.c_maintenance * general_params.l_round_trip /1000;
output.c_M_round_trip = c_M_round_trip;
fprintf('Cost of maintenance: \n %1.2f SEK \n', c_M_round_trip);
% TODO: Cost of maintenance
% TODO: Efficiency
% TODO: Normalized equipment costs (life cycle)
end