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

pass_flow_rest_A2B = pass_flow_A2B - tram_freq * tram_params.n_pass;
temp = pass_flow_rest_A2B < 0;
pass_flow_rest_A2B(temp) = 0;

pass_flow_rest_B2A = pass_flow_B2A - tram_freq * tram_params.n_pass;
temp = pass_flow_rest_B2A < 0;
pass_flow_rest_B2A(temp) = 0;
output.pass_flow_rest = pass_flow_rest;

car_n_pass = car_params.n_pass;
car_trips_A2B = ceil(pass_flow_rest_A2B / car_n_pass);
car_trips_B2A = ceil(pass_flow_rest_B2A / car_n_pass);

% Passenger flow rest rounded up to the nearest car_n_pass, since the
% number of trips within an hour should be an integer
pass_flow_rest_A2B_car_capacity = car_trips_A2B * car_n_pass;
pass_flow_rest_B2A_car_capacity = car_trips_B2A * car_n_pass;

% All cars come back to A at the end of day 
% Flow capacity A start: flow capacity at the start of hour
% Flow capacity A end: flow capacity at the end of hour

[m , n] = size(pass_flow_rest);
car_flow_capacity_A_start =  num_cars_grid *  car_params.n_pass;
car_flow_capacity_A_end = zeros(m,n);
car_flow_capacity_B_start = zeros(m,n);
car_flow_capacity_B_end = car_flow_capacity_B_start;
car_empty_trips_B2A = zeros(m,n);
car_empty_trips_A2B = zeros(m,n);
car_unused_capacity_A = zeros(m,n);

for i = 1:m
    for j = 1:n
        car_flow_capacity_A_end(i,j) = car_flow_capacity_A_start(i,j) - pass_flow_rest_A2B_car_capacity(i,j) + pass_flow_rest_B2A_car_capacity(i,j);
        car_flow_capacity_B_end(i,j) = car_flow_capacity_A_start(i,1) - car_flow_capacity_A_end(i,j);
        if j < n
            if car_flow_capacity_A_end(i,j) < pass_flow_rest_A2B_car_capacity(i,j+1)
                car_empty_trips_B2A(i,j) = ((pass_flow_rest_A2B_car_capacity(i,j+1) - car_flow_capacity_A_end(i,j))/car_n_pass);
                car_flow_capacity_A_end(i,j) = car_flow_capacity_A_start(i,j) - pass_flow_rest_A2B_car_capacity(i,j) + pass_flow_rest_B2A_car_capacity(i,j) + car_empty_trips_B2A(i,j)* car_n_pass;
                car_flow_capacity_B_end(i,j) = car_flow_capacity_A_start(i,1) - car_flow_capacity_A_end(i,j);
                
            elseif pass_flow_rest_A2B_car_capacity(i,j) < pass_flow_rest_B2A_car_capacity(i,j) && car_flow_capacity_B_end(i,j) < pass_flow_rest_B2A_car_capacity(i,j+1)
                car_empty_trips_A2B(i,j) = ((pass_flow_rest_B2A_car_capacity(i,j+1) - car_flow_capacity_B_end(i,j))/car_n_pass);
                car_flow_capacity_A_end(i,j) = car_flow_capacity_A_start(i,j) - pass_flow_rest_A2B_car_capacity(i,j) + pass_flow_rest_B2A_car_capacity(i,j) - car_empty_trips_A2B(i,j)* car_n_pass;
                car_flow_capacity_B_end(i,j) = car_flow_capacity_A_start(i,1) - car_flow_capacity_A_end(i,j);
            end
            car_flow_capacity_A_start(i,j+1) = car_flow_capacity_A_end(i,j);
            car_flow_capacity_B_start(i,j+1) = car_flow_capacity_B_end(i,j);
        else
            car_empty_trips_B2A(i,j) = car_flow_capacity_B_end(i,j)/car_n_pass;
            car_flow_capacity_A_end(i,j) = car_flow_capacity_A_end(i,j) + car_empty_trips_B2A(i,j)*car_n_pass;
            car_flow_capacity_B_end(i,j) = car_flow_capacity_B_end(i,j) - car_empty_trips_B2A(i,j)*car_n_pass;
        end              
    end
end

for i = 1:m
   for j = 1:n
       if j < n
           % Cars available to charge at A. No cars will be available for
           % charge at B as the capacity is maintained to be only sufficient
           % for the next hour trip
           if car_flow_capacity_A_end(i,j) > pass_flow_rest_A2B_car_capacity(i,j+1)
               car_unused_capacity_A(i,j) = car_flow_capacity_A_end(i,j) - pass_flow_rest_A2B_car_capacity(i,j+1);
           end
       else
           car_unused_capacity_A(i,j) = car_flow_capacity_A_end(i,j);
       end
   end
end

car_round_trips = min(car_trips_A2B, car_trips_B2A);

% Car frequency
car_freq = ceil(pass_flow_rest / car_params.n_pass);
output.car_freq = car_freq;

output.car_flow_capacity_A_end = car_flow_capacity_A_end;
output.car_flow_capacity_B_end = car_flow_capacity_B_end;
output.car_flow_capacity_A_start = car_flow_capacity_A_start;
output.car_flow_capacity_B_start = car_flow_capacity_B_start;
output.car_empty_trips_A2B = car_empty_trips_A2B;
output.car_empty_trips_B2A = car_empty_trips_B2A;
output.car_trips_A2B = car_trips_A2B;
output.car_trips_B2A = car_trips_B2A;
output.car_round_trips = car_round_trips;
output.car_unused_capacity_A = car_unused_capacity_A;

% Minimum number of car and tram chargers
% NOTE: This currently uses the C-rate to calculate charging time, but
% it might be better to use charging station power instead if it's better
% to have it lower than the C-rate. Also, having that number makes it
% cleaner to calculate the charging station cost (which currently is
% calculated separately in the cost estimation function using C-rate)

% NOTE: This needs car_full_trips_A2B, car_full_trips_B2A, 
% car_empty_trips_A2B and car_empty_trips_B2A to be integers.
[n_car_chargers_A, n_car_chargers_B, car_energy_charged_grid, ...
    car_energy_discharged_grid, car_energy_remaining_grid, ...
    car_constraint_compliance] = min_number_of_chargers(general_params, ...
    car_params, car_trips_A2B, car_trips_B2A, car_empty_trips_A2B, ...
    car_empty_trips_B2A, num_cars, time_hr, n_variations_adjusted, ...
    time_per_round_trip_car * 60);

% NOTE: The function call below is using car_freq as a placeholder for the
% trips in each direction. Since it represents the number of round trips,
% simply doubling it give the number of trips in each direction, which also
% means that the vehicles never stay in destination B
%[n_car_chargers_A, n_car_chargers_B, car_energy_charged_grid, ...
%    car_energy_discharged_grid, car_energy_remaining_grid, ...
%    car_constraint_compliance] = min_number_of_chargers(general_params, ...
%    car_params, car_freq, car_freq, zeros(size(car_freq)), ...
%    zeros(size(car_freq)), num_cars, time_hr, n_variations_adjusted, ...
%    time_per_round_trip_car * 60);

output.n_car_chargers_A = n_car_chargers_A;
output.n_car_chargers_B = n_car_chargers_B;
output.n_car_chargers = n_car_chargers_A + n_car_chargers_B;
output.P_car_charger_kW = car_params.E_battery_size_kWh * general_params.C_rate;
output.P_car_chargers = output.P_car_charger_kW * output.n_car_chargers;

no_car_idx = find(num_cars == 0);
car_energy_charged_grid(no_car_idx,:) = 0;
car_energy_discharged_grid(no_car_idx,:) = 0;
car_energy_remaining_grid(no_car_idx,:) = 0;

output.car_energy_charged_grid = car_energy_charged_grid;
output.car_energy_discharged_grid = car_energy_discharged_grid;
output.car_energy_remaining_grid = car_energy_remaining_grid;
output.car_constraint_compliance = car_constraint_compliance;

% NOTE: The function call below is using tram_freq as a placeholder for the
% trips in each direction. Since it represents the number of round trips,
% simply doubling it give the number of trips in each direction, which also
% means that the vehicles never stay in destination B
[n_tram_chargers_A, n_tram_chargers_B, tram_energy_charged_grid, ...
    tram_energy_discharged_grid, tram_energy_remaining_grid, ...
    tram_constraint_compliance] = min_number_of_chargers(general_params, ...
    tram_params, tram_freq, tram_freq, zeros(size(tram_freq)), ...
    zeros(size(tram_freq)), num_trams, time_hr, n_variations_adjusted, ...
    time_per_round_trip_tram * 60);
output.n_tram_chargers_A = n_tram_chargers_A;
output.n_tram_chargers_B = n_tram_chargers_B;
output.n_tram_chargers = n_tram_chargers_A + n_tram_chargers_B;
output.P_tram_charger_kW = tram_params.E_battery_size_kWh * general_params.C_rate;
output.P_tram_chargers = output.P_tram_charger_kW * output.n_tram_chargers;

no_tram_idx = find(num_trams == 0);
tram_energy_charged_grid(no_tram_idx,:) = 0;
tram_energy_discharged_grid(no_tram_idx,:) = 0;
tram_energy_remaining_grid(no_tram_idx,:) = 0;

output.tram_energy_charged_grid = tram_energy_charged_grid;
output.tram_energy_discharged_grid = tram_energy_discharged_grid;
output.tram_energy_remaining_grid = tram_energy_remaining_grid;
output.tram_constraint_compliance = tram_constraint_compliance;
output.car_constraint_compliance = car_constraint_compliance;
output.fleet_constraint_compliance = tram_constraint_compliance ...
    & car_constraint_compliance;
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

function [ n_chargers_A, n_chargers_B, energy_charged_grid, energy_discharged_grid, ...
    energy_remaining_grid, constraint_compliance ] = min_number_of_chargers ...
    (general_params, vehicle_params, vehicle_full_trips_A2B, ...
    vehicle_full_trips_B2A, vehicle_empty_trips_A2B, vehicle_empty_trips_B2A, ...
    num_vehicles, time_hr, n_variations, t_round_trip)
%MIN_NUMBER_OF_CHARGERS Calculates the minimum amount of chargers required
%   For this vehicle type, calculate the minimum number of chargers
%   required fo each mix of vehicles

n_chargers_A = NaN(size(num_vehicles));
n_chargers_B = NaN(size(num_vehicles));
energy_charged_grid = NaN(size(vehicle_full_trips_A2B));
energy_discharged_grid = NaN(size(vehicle_full_trips_A2B));
energy_remaining_grid = NaN(size(vehicle_full_trips_A2B));
constraint_compliance = false(size(num_vehicles));
for i = 1:n_variations
    if 0 == num_vehicles(i)
       n_chargers_A(i) = 0;
       n_chargers_B(i) = 0;
       constraint_compliance(i) = true;
       continue;
    end
    
    % Stepwise linear search to find minimum number of chargers (the
    % minimum sum of chargers at destination A and B)
    % The assumption is that a valid number of chargers exist in the search
    % space, i.e. at the maximum of one charger per vehicle at each
    % destination
    chargers_A = num_vehicles(i);
    chargers_B = 0;
    while (chargers_A >= 0 && chargers_B <= num_vehicles(i))
        [ok, energy_charged_hr, energy_discharged_hr, energy_remaining_hr] = ...
            check_number_of_chargers(general_params, vehicle_params, ...
            vehicle_full_trips_A2B(i,:), vehicle_full_trips_B2A(i,:), ...
            vehicle_empty_trips_A2B(i, :), vehicle_empty_trips_B2A(i, :), ...
            num_vehicles(i), time_hr, chargers_A, chargers_B, t_round_trip);
        if (ok)
            n_chargers_A(i) = chargers_A;
            n_chargers_B(i) = chargers_B;
            constraint_compliance(i) = true;
            energy_charged_grid(i,:) = energy_charged_hr;
            energy_discharged_grid(i,:) = energy_discharged_hr;
            energy_remaining_grid(i,:) = energy_remaining_hr;
            chargers_A = chargers_A - 1;
        else
            chargers_A = chargers_A - 1;
            chargers_B = chargers_B + 1;
        end
    end
end
end

function [ ok, energy_charged_hr, energy_discharged_hr, energy_remaining_hr ] = ...
    check_number_of_chargers (general_params, vehicle_params, vehicle_full_trips_A2B, ...
    vehicle_full_trips_B2A, vehicle_empty_trips_A2B, vehicle_empty_trips_B2A, ...
    num_vehicles, time_hr, chargers_A, chargers_B, t_round_trip)
%CHECK_NUMBER_OF_CHARGERS Checks if this number of chargers is enough
%   Checks if this number of chargers is enough to perform all the
%   required vehicle trips and recharge all vehicles completely until
%   the next day.
%   The charging strategy used is to drive the vehicles with the most
%   energy left and charge the vehicles with the least energy left. This
%   is reevaluated after each round trip. In addition, the during a round
%   trip iteration, if a vehicle is fully charged, another vehicle may
%   continue to use that charger for the remaining time.
dest_A = 1;
dest_B = 2;

% TODO: Use single trip energy instead
% TODO: Use full / empty trip energy instead
E_single_trip = vehicle_params.E_round_trip_kWh / 2;
t_single_trip = t_round_trip / 2;
energy_charged_hr = zeros(numel(time_hr), 1);
energy_discharged_hr = zeros(numel(time_hr), 1);
energy_remaining_hr = zeros(numel(time_hr), 1);
vehicles = ones(num_vehicles,2);
vehicles(:,1) = ones(num_vehicles,1) * vehicle_params.E_battery_size_kWh;
prev_hr = time_hr(1) - 1;
for i = 1:numel(time_hr)
    % NOTE: Vehicles are sorted in ascending order by remaining energy
    vehicles_A = vehicles(vehicles(:,2) == dest_A,:);
    vehicles_B = vehicles(vehicles(:,2) == dest_B,:);
    
    % Special case: Gap of more than one hour detected, charge accordingly
    curr_hr = time_hr(i);
    if curr_hr > prev_hr + 1
        diff_s = (curr_hr - prev_hr - 1) * 3600;
        
        % Charge vehicles at destination A
        [vehicles_A, energy_charged] = update_charging(vehicles_A, ...
            vehicle_params.E_battery_size_kWh, general_params.C_rate, ...
            chargers_A, 0, diff_s);
        
        % Update the energy charged output
        energy_charged_hr(i) = energy_charged_hr(i) + energy_charged;
        
        % Charge vehicles at destination B
        [vehicles_B, energy_charged] = update_charging(vehicles_B, ...
            vehicle_params.E_battery_size_kWh, general_params.C_rate, ...
            chargers_B, 0, diff_s);
        
        % Update the energy charged output
        energy_charged_hr(i) = energy_charged_hr(i) + energy_charged;
        
        % Sort the vehicle matrices
        vehicles_A = sort(vehicles_A, 1);
        vehicles_B = sort(vehicles_B, 1);
    end
    
    % For each round trip time, charge vehicles with the least
    % amount of remaining energy, and discharge (use) vehicles with
    % the most amount of remaining energy. If discharge results in empty
    % battery, this number of chargers is not enough for the daily commute.
    iterations = floor(3600 / t_single_trip);
    completed_iterations = 0;
    hr_vehicles_A2B = vehicle_full_trips_A2B(i) + vehicle_empty_trips_A2B(i);
    hr_vehicles_B2A = vehicle_full_trips_B2A(i) + vehicle_empty_trips_B2A(i);
    for j = 1:iterations
        n_vehicles_A = size(vehicles_A, 1);
        n_vehicles_B = size(vehicles_B, 1);
        trip_vehicles_A = min(n_vehicles_A, hr_vehicles_A2B);
        trip_vehicles_B = min(n_vehicles_B, hr_vehicles_B2A);
        hr_vehicles_A2B = max(0, hr_vehicles_A2B - trip_vehicles_A);
        hr_vehicles_B2A = max(0, hr_vehicles_B2A - trip_vehicles_B);
        
        % Charge vehicles at destination A
        [vehicles_A, energy_charged] = update_charging(vehicles_A, ...
            vehicle_params.E_battery_size_kWh, general_params.C_rate, ...
            chargers_A, trip_vehicles_A, t_single_trip);
        
        % Discharge vehicles travelling from A to B
        [vehicles_A, ok, energy_discharged] = update_discharging(vehicles_A, ...
            E_single_trip, trip_vehicles_A);
        
        % Update the energy charged / discharged outputs for destination A
        energy_charged_hr(i) = energy_charged_hr(i) + energy_charged;
        energy_discharged_hr(i) = energy_discharged_hr(i) + energy_discharged;
        if ~ok
            % We break here, but still want to have the correct energy
            % remaining, for debugging purposes
            energy_remaining_hr(i) = sum(vehicles_A(:,1)) + sum(vehicles_B(:,1));
            return;
        end
        
         % Charge vehicles at destination B
        [vehicles_B, energy_charged] = update_charging(vehicles_B, ...
            vehicle_params.E_battery_size_kWh, general_params.C_rate, ...
            chargers_B, trip_vehicles_B, t_single_trip);
        
        % Discharge vehicles travelling from B to A
        % TODO: Use single trip energy instead
        % TODO: Use full / empty trip energy instead
        [vehicles_B, ok, energy_discharged] = update_discharging(vehicles_B, ...
            E_single_trip, trip_vehicles_B);
        
        % Update the energy charged / discharged outputs for destination A
        energy_charged_hr(i) = energy_charged_hr(i) + energy_charged;
        energy_discharged_hr(i) = energy_discharged_hr(i) + energy_discharged;
        if ~ok
            % We break here, but still want to have the correct energy
            % remaining, for debugging purposes
            energy_remaining_hr(i) = sum(vehicles_A(:,1)) + sum(vehicles_B(:,1));
            return;
        end
        
        % Redistribute vehicles
        vehicles_A_remaining = vehicles_A(1:end - trip_vehicles_A,:);
        vehicles_A_leaving = vehicles_A(end - trip_vehicles_A + 1:end,:);
        vehicles_B_remaining = vehicles_B(1:end - trip_vehicles_B,:);
        vehicles_B_leaving = vehicles_B(end - trip_vehicles_B + 1:end,:);
        vehicles_A = [vehicles_A_remaining; vehicles_B_leaving];
        vehicles_B = [vehicles_B_remaining; vehicles_A_leaving];
        vehicles_A(:,2) = dest_A;
        vehicles_B(:,2) = dest_B;
        
        % Sort the vehicle matrices
        vehicles_A = sort(vehicles_A, 1);
        vehicles_B = sort(vehicles_B, 1);
        completed_iterations = j;
        
        % No more required trips
        if (hr_vehicles_A2B == 0 && hr_vehicles_B2A == 0)
            break;
        end
    end
    
    % More required trips than there are vehicles!
    if (hr_vehicles_A2B > 0 || hr_vehicles_B2A > 0)
        throw(MException('FLEET_SIZE_ESTIMATION:CHECK_NUMBER_OF_CHARGERS:RequiredTripsExceedCapacity',...
            'More required trips than there are vehicles!'));
    end
    
    % Remainder of hour spent charging all vehicles
    t_remaining = max(0, 3600 - completed_iterations * t_single_trip);
    
    % Charge vehicles at destination A
    [vehicles_A, energy_charged] = update_charging(vehicles_A, ...
        vehicle_params.E_battery_size_kWh, general_params.C_rate, ...
        chargers_A, 0, t_remaining);
    
    % Update the energy charged output
    energy_charged_hr(i) = energy_charged_hr(i) + energy_charged;
    
    % Charge vehicles at destination B
    [vehicles_B, energy_charged] = update_charging(vehicles_B, ...
        vehicle_params.E_battery_size_kWh, general_params.C_rate, ...
        chargers_B, 0, t_remaining);
    
    % Update the energy charged output
    energy_charged_hr(i) = energy_charged_hr(i) + energy_charged;
    
    % Update the energy remaining outputs
    energy_remaining_hr(i) = sum(vehicles_A(:,1)) + sum(vehicles_B(:,1));
    prev_hr = curr_hr;
    
    % Redistribute and sort the vehicle matrices
    vehicles = sort([vehicles_A; vehicles_B], 1);
end

% Extract vehicles at each destination
vehicles_A = vehicles(vehicles(:,2) == dest_A,:);
vehicles_B = vehicles(vehicles(:,2) == dest_B,:);

% No vehicles should remain at B
if (size(vehicles_B, 1) > 0)
    throw(MException('FLEET_SIZE_ESTIMATION:CHECK_NUMBER_OF_CHARGERS:SomeVehiclesNotReturned',...
        'Some vehicles did not return to destination A at the end of the day!'));
end

% Need to be able to fully charge during night (time between the end of the
% last hour and the beginning of the first hour in the passenger flow)
t_night = 3600 * max(0,(min(time_hr) + 23 - max(time_hr)));
vehicles_A = update_charging(vehicles_A, vehicle_params.E_battery_size_kWh, ...
    general_params.C_rate, chargers_A, 0, t_night);
ok = vehicle_params.E_battery_size_kWh <= min(vehicles_A(:,1));
end

function [ vehicles, energy_charged ] = update_charging(vehicles, E_max, ...
    C_rate, n_chargers, n_excluded_vehicles, t_charging)
%UPDATE_CHARGING Charge specified vehicles
%   'vehicles' has to be sorted in ascending order

energy_charged = 0;
n_vehicles = size(vehicles, 1);
if (0 == n_chargers) || (0 == n_vehicles) ...
        || (n_vehicles == n_excluded_vehicles)
    return;
end
t_charge_full = 3600 / C_rate;
E_diff_max_charger = (t_charging / t_charge_full) * E_max;
E_chargers_tot = E_diff_max_charger * n_chargers;

% Each vehicle may charge 't_charging' amount of time in total, but if that
% is not required in order to fully charge vehicle, some other vehicle may
% use the remaining time at that charger
for i = 1:(n_vehicles - n_excluded_vehicles)
    E_vehicle = vehicles(i, 1);
    E_diff_max_vehicle = max(0, E_max - E_vehicle);
    E_diff_actual = min(E_chargers_tot, min(E_diff_max_charger, ...
        E_diff_max_vehicle));
    vehicles(i, 1) = E_vehicle + E_diff_actual;
    energy_charged = energy_charged + E_diff_actual;
    E_chargers_tot = max(0, E_chargers_tot - E_diff_actual);
    if 0 >= E_chargers_tot
       break;
    end
end
end

function [ vehicles, ok, energy_discharged ] = update_discharging(vehicles, ...
    E_trip, n_trip_vehicles)
%UPDATE_DISCHARGING Discharge the specified vehicles
%   'vehicles' has to be sorted in ascending order

ok = true;
energy_discharged = 0;
n_vehicles = size(vehicles, 1);
if (0 == n_trip_vehicles || 0 == n_vehicles)
    return;
end
for i = (n_vehicles - n_trip_vehicles + 1):n_vehicles
    E_diff_actual = min(vehicles(i, 1), E_trip);
    vehicles(i, 1) = max(0, vehicles(i, 1) - E_diff_actual);
    energy_discharged = energy_discharged + E_diff_actual;
    if 0 >= vehicles(i, 1)
        ok = false;
    end
end
end
