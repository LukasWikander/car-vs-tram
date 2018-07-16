function [ output ] = fleet_size_estimation( tram_params, car_params, general_params, drv_mission, pass_flow, n_variations )
%FLEET_SIZE_ESTIMATION Summary of this function goes here
%   Detailed explanation goes here

output = struct;

%% Driving mission
% The driving mission is based on slope [rad] with respect to distance
slope        = drv_mission.dc.slope;
distance     = drv_mission.dc.s;

%% Passenger flow
% The number of passengers travelling from A to B during different time of
% the day
time_hr      = pass_flow.x;
from_A2B     = pass_flow.yA;
from_B2A     = pass_flow.yB;

%% Fleet size
mean_flow     = sum (from_A2B) / (max(time_hr) - min(time_hr));
mean_flow     = floor(mean_flow); 
output.mean_flow = mean_flow;

% One way trip length [min]
time_per_trip_tram      = tram_params.t_round_trip / 2 / 60; 
% Round trip length [min]
time_per_round_trip_tram  = 2 * time_per_trip_tram + max(2 * tram_params.t_unload / 60, tram_params.t_charging_round_trip / 60);
% Number of round trips per hour each tram
num_round_trip_hr_tram  = floor(60 / time_per_round_trip_tram);
% Maximum number of trams
max_flow_hr             = max(max(from_A2B),max(from_B2A));
max_num_trams           = ceil(max_flow_hr / tram_params.n_pass / num_round_trip_hr_tram);
n_variations_adjusted   = min([n_variations, max_num_trams + 1]); % Question: why does it need to be adjusted?
% Number of trams
num_trams               = floor(linspace(0,max_num_trams,n_variations_adjusted));
output.num_trams = num_trams;
% Flow capacity of tram line
flow_cap_hr_tram        = tram_params.n_pass * num_round_trip_hr_tram .* num_trams;

% One way trip length [min]
time_per_trip_car       = car_params.t_round_trip / 60;  
% Round trip length [min]
time_per_round_trip_car   = 2 * time_per_trip_car + max(2 * car_params.t_unload / 60, car_params.t_charging_round_trip / 60);
% Number of round trips per hour each car
num_round_trip_hr_car   = floor(60 / time_per_round_trip_car);   
% Number of cars in fleet
num_cars                 = ceil((max(0, max(from_A2B) - flow_cap_hr_tram)) / car_params.n_pass / num_round_trip_hr_car);
output.num_cars = num_cars;

% Total flow capacity
flow_cap_hr_total       = flow_cap_hr_tram + num_cars * car_params.n_pass * num_round_trip_hr_car;
output.flow_cap_hr_total = flow_cap_hr_total;

%% Fleet planning

num_trams_grid = meshgrid(num_trams, time_hr);
num_trams_grid = transpose(num_trams_grid);
output.num_trams_grid = num_trams_grid;

num_cars_grid = meshgrid(num_cars, time_hr);
num_cars_grid = transpose(num_cars_grid);
output.num_cars_grid = num_cars_grid;

pass_flow_A2B = transpose(from_A2B);
pass_flow_A2B = repmat(pass_flow_A2B, n_variations_adjusted, 1);
output.pass_flow_A2B = pass_flow_A2B;

pass_flow_B2A = transpose(from_B2A);
pass_flow_B2A = repmat(pass_flow_B2A, n_variations_adjusted, 1);
output.pass_flow_B2A = pass_flow_B2A;

pass_flow = max(pass_flow_A2B, pass_flow_B2A);
output.pass_flow = pass_flow;

% Tram frequency from A to B
tram_freq_A2B = vehicle_frequence(tram_params, pass_flow_A2B, num_round_trip_hr_tram, num_trams_grid, time_hr, n_variations_adjusted);
output.tram_freq_A2B = tram_freq_A2B;
% Tram frequency from B to A
tram_freq_B2A = vehicle_frequence(tram_params, pass_flow_B2A, num_round_trip_hr_tram, num_trams_grid, time_hr, n_variations_adjusted);
output.tram_freq_B2A = tram_freq_B2A;

% Tram frequency including tram returning empty
tram_freq = max(tram_freq_A2B, tram_freq_B2A);
utilization_tram = round ( 100 * tram_freq ./ num_trams_grid /num_round_trip_hr_tram );
empty_tram_A2B = tram_freq - tram_freq_A2B;
empty_tram_B2A = tram_freq - tram_freq_B2A;
output.tram_freq = tram_freq;
output.utilization_tram = utilization_tram;
output.empty_tram_A2B = empty_tram_A2B;
output.empty_tram_B2A = empty_tram_B2A;

% Rest passenger flow
pass_flow_rest = max(0, pass_flow - tram_freq * tram_params.n_pass);
output.pass_flow_rest = pass_flow_rest;

% Car frequency
car_freq = ceil(pass_flow_rest / car_params.n_pass);
unused_cars = num_cars_grid - car_freq;
output.car_freq = car_freq;
output.unused_cars = unused_cars;

% Minimum number of car and tram chargers
% NOTE: This currently uses the C-rate to calculate charging time, but
% it might be better to use charging station power instead if it's better
% to have it lower than the C-rate. Also, having that number makes it
% cleaner to calculate the charging station cost (which currently is
% calculated separately in the cost estimation function using C-rate)
[n_car_chargers, car_energy_charged_grid, car_energy_discharged_grid, ...
    car_energy_remaining_grid] = min_number_of_chargers(general_params, ...
    car_params, car_freq, num_cars, time_hr, n_variations_adjusted, ...
    time_per_round_trip_car * 60);
output.n_car_chargers = n_car_chargers;
output.P_car_charger_kW = car_params.E_battery_size_kWh * general_params.C_rate;
output.P_car_chargers = output.P_car_charger_kW * n_car_chargers;
output.car_energy_charged_grid = car_energy_charged_grid;
output.car_energy_discharged_grid = car_energy_discharged_grid;
output.car_energy_remaining_grid = car_energy_remaining_grid;
[n_tram_chargers, tram_energy_charged_grid, tram_energy_discharged_grid, ...
    tram_energy_remaining_grid] = min_number_of_chargers(general_params, ...
    tram_params, tram_freq, num_trams, time_hr, n_variations_adjusted, ...
    time_per_round_trip_tram * 60);
output.n_tram_chargers = n_tram_chargers;
output.P_tram_charger_kW = tram_params.E_battery_size_kWh * general_params.C_rate;
output.P_tram_chargers = output.P_tram_charger_kW * n_tram_chargers;
output.tram_energy_charged_grid = tram_energy_charged_grid;
output.tram_energy_discharged_grid = tram_energy_discharged_grid;
output.tram_energy_remaining_grid = tram_energy_remaining_grid;
end

function [ freq ] = vehicle_frequence (vehicle_params, num_pass, ...
    num_round_trip_hr, num_vehicles_grid, time_hr, n_variations)
%VEHICLE_FREQUENCE Calculates the number of round trips per hour and mix
%   For this vehicle type, this function calculates the number required
%   number of round trips for each hour and mix of vehicles

flow = num_pass;
freq = zeros(n_variations,length(time_hr));
for i = 2:n_variations
    for  j = 1:length(time_hr)
        if (flow(i,j) < vehicle_params.n_pass) && (flow(i,j) > 0)
            freq(i,j) = 1;
        elseif (flow(i,j)> vehicle_params.n_pass) && (flow(i,j) ...
                < vehicle_params.n_pass * num_round_trip_hr)
            freq(i,j) = num_round_trip_hr ;
        elseif flow(i,j)>= vehicle_params.n_pass * num_round_trip_hr ...
                * num_vehicles_grid(i,j)
            freq(i,j) = num_round_trip_hr * num_vehicles_grid(i,j);
        else
            freq(i,j) = ceil(flow(i,j) / vehicle_params.n_pass);
        end
    end
end
end

function [ n_chargers, energy_charged_grid, energy_discharged_grid, ...
    energy_remaining_grid ] = min_number_of_chargers (general_params, ...
    vehicle_params, vehicle_freq, num_vehicles, time_hr, n_variations, ...
    t_round_trip)
%MIN_NUMBER_OF_CHARGERS Calculates the minimum amount of chargers required
%   For this vehicle type, calculate the minimum number of chargers
%   required fo each mix of vehicles
%   Assumption: All chargers situated at exactly one of the destinations,
%   to simplify evaluation

n_chargers = num_vehicles;
energy_charged_grid = zeros(size(vehicle_freq));
energy_discharged_grid = zeros(size(vehicle_freq));
energy_remaining_grid = zeros(size(vehicle_freq));
for i = 1:n_variations
    if 0 == num_vehicles(i)
       n_chargers(i) = 0;
       continue;
    end
    
    % Binary search to find minimum number of chargers
    % The assumption is that a valid number of chargers exist in the search
    % space, i.e. at the maximum of one charger per vehicle
    left = 0;
    right = n_chargers(i);
    while (left <= right)
        mid = floor((left + right) / 2);
        [ok, energy_charged_hr, energy_discharged_hr, energy_remaining_hr] = ...
            check_number_of_chargers(general_params, vehicle_params, ...
            vehicle_freq(i,:), num_vehicles(i), time_hr, mid, t_round_trip)
        if (ok)
            right = mid - 1;
            n_chargers(i) = mid;
            energy_charged_grid(i,:) = energy_charged_hr;
            energy_discharged_grid(i,:) = energy_discharged_hr;
            energy_remaining_grid(i,:) = energy_remaining_hr;
        else
            left = mid + 1;
        end
    end
end
end

function [ ok, energy_charged_hr, energy_discharged_hr, energy_remaining_hr ] = ...
    check_number_of_chargers (general_params, vehicle_params, vehicle_freq, ...
    num_vehicles, time_hr, number_of_chargers, t_round_trip)
%CHECK_NUMBER_OF_CHARGERS Checks if this number of chargers is enough
%   Checks if this number of chargers is enough to perform all the
%   required vehicle trips and recharge all vehicles completely until
%   the next day.
%   The charging strategy used is to drive the vehicles with the most
%   energy left and charge the vehicles with the least energy left. This
%   is reevaluated after each round trip. In addition, the during a round
%   trip iteration, if a vehicle is fully charged, another vehicle may
%   continue to use that charger for the remaining time.

energy_charged_hr = zeros(numel(time_hr), 1);
energy_discharged_hr = zeros(numel(time_hr), 1);
energy_remaining_hr = zeros(numel(time_hr), 1);
vehicles = ones(num_vehicles,1) * vehicle_params.E_battery_size_kWh;
prev_hr = time_hr(1) - 1;
for i = 1:numel(time_hr)
    % NOTE: Vehicles are sorted in ascending order by remaining energy
    
    % Special case: Gap of more than one hour detected, charge accordingly
    curr_hr = time_hr(i);
    if curr_hr > prev_hr + 1
        diff_s = (curr_hr - prev_hr - 1) * 3600;
        [vehicles, energy_charged] = update_charging(vehicles, ...
            vehicle_params.E_battery_size_kWh, general_params.C_rate, ...
            number_of_chargers, 0, diff_s);
        vehicles = sort(vehicles);
        
        % Update the energy charged output
        energy_charged_hr(i) = energy_charged_hr(i) + energy_charged;
    end
    
    % For each round trip time, charge vehicles with the least
    % amount of remaining energy, and discharge (use) vehicles with
    % the most amount of remaining energy. If discharge results in empty
    % battery, this number of chargers is not enough for the daily commute.
    iterations = ceil(vehicle_freq(i) / num_vehicles);
    for j = 1:iterations
        trip_vehicles = min(num_vehicles, vehicle_freq(i) - (j-1)*num_vehicles);
        [vehicles, energy_charged] = update_charging(vehicles, ...
            vehicle_params.E_battery_size_kWh, general_params.C_rate, ...
            number_of_chargers, trip_vehicles, t_round_trip);
        [vehicles, ok, energy_discharged] = update_discharging(vehicles, ...
            vehicle_params.E_round_trip_kWh, trip_vehicles);
        
        % Update the energy charged / discharged outputs
        energy_charged_hr(i) = energy_charged_hr(i) + energy_charged;
        energy_discharged_hr(i) = energy_discharged_hr(i) + energy_discharged;
        if ~ok
            % We break here, but still want to have the correct energy
            % remaining, for debugging purposes
            energy_remaining_hr(i) = sum(vehicles);
            return;
        end
        vehicles = sort(vehicles);
    end
    
    % Remainder of hour spent charging all vehicles
    t_remaining = max(0, 3600 - iterations * t_round_trip);
    [vehicles, energy_charged] = update_charging(vehicles, ...
        vehicle_params.E_battery_size_kWh, general_params.C_rate, ...
        number_of_chargers, 0, t_remaining);
    vehicles = sort(vehicles);
    
    % Update the energy charged / remaining outputs
    energy_charged_hr(i) = energy_charged_hr(i) + energy_charged;
    energy_remaining_hr(i) = sum(vehicles);
    prev_hr = curr_hr;
end

% Need to be able to fully charge during night (time between the end of the
% last hour and the beginning of the first hour in the passenger flow)
t_night = 3600 * max(0,(min(time_hr) + 23 - max(time_hr)));
vehicles = update_charging(vehicles, vehicle_params.E_battery_size_kWh, ...
    general_params.C_rate, number_of_chargers, 0, t_night);
ok = vehicle_params.E_battery_size_kWh <= min(vehicles);
end

function [ vehicles, energy_charged ] = update_charging(vehicles, E_max, ...
    C_rate, n_chargers, n_excluded_vehicles, t_charging)
%UPDATE_CHARGING Charge specified vehicles
%   'vehicles' has to be sorted in ascending order

energy_charged = 0;
n_vehicles = numel(vehicles);
if (0 == n_chargers) || (n_vehicles == n_excluded_vehicles)
    return;
end
t_charge_full = 3600 / C_rate;
E_diff_max_charger = (t_charging / t_charge_full) * E_max;
E_chargers_tot = E_diff_max_charger * n_chargers;

% Each vehicle may charge 't_charging' amount of time in total, but if that
% is not required in order to fully charge vehicle, some other vehicle may
% use the remaining time at that charger
for i = 1:(n_vehicles - n_excluded_vehicles)
    E_vehicle = vehicles(i);
    E_diff_max_vehicle = max(0, E_max - E_vehicle);
    E_diff_actual = min(E_chargers_tot, min(E_diff_max_charger, ...
        E_diff_max_vehicle));
    vehicles(i) = E_vehicle + E_diff_actual;
    energy_charged = energy_charged + E_diff_actual;
    E_chargers_tot = max(0, E_chargers_tot - E_diff_actual);
    if 0 >= E_chargers_tot
       break;
    end
end
end

function [ vehicles, ok, energy_discharged ] = update_discharging(vehicles, ...
    E_rt, n_trip_vehicles)
%UPDATE_DISCHARGING Discharge the specified vehicles
%   'vehicles' has to be sorted in ascending order

ok = true;
energy_discharged = 0;
n_vehicles = numel(vehicles);
if 0 == n_trip_vehicles
    return
end;
for i = (n_vehicles - n_trip_vehicles + 1):n_vehicles
    E_diff_actual = min(vehicles(i), E_rt);
    vehicles(i) = max(0, vehicles(i) - E_diff_actual);
    energy_discharged = energy_discharged + E_diff_actual;
    if 0 >= vehicles(i)
        ok = false;
    end
end
end
