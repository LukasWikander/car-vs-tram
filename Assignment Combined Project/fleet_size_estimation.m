function [ output ] = fleet_size_estimation( tram_params, car_params, drv_mission, pass_flow, n_variations )
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
time_per_round_trip_tram  = 2 * time_per_trip_tram + 2 * tram_params.t_unload / 60 + tram_params.t_charging_round_trip / 60;
% Number of round trips per hour each tram
num_round_trip_hr_tram  = floor(60 / time_per_round_trip_tram);
% Maximum number of trams
max_flow_hr             = max(max(from_A2B),max(from_B2A));
max_num_trams           = ceil(max_flow_hr / tram_params.n_pass / num_round_trip_hr_tram);
n_variations_adjusted   = min([n_variations - 1, max_num_trams]); % Question: why does it need to be adjusted?
% Number of trams
num_trams               = floor(linspace(0,max_num_trams,n_variations_adjusted));
output.num_trams = num_trams;
% Flow capacity of tram line
flow_cap_hr_tram        = tram_params.n_pass * num_round_trip_hr_tram .* num_trams;

% One way trip length [min]
time_per_trip_car       = car_params.t_round_trip / 60;  
% Round trip length [min]
time_per_round_trip_car   = 2 * time_per_trip_car + 2 * car_params.t_unload / 60 + car_params.t_charging_round_trip / 60;
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

end

function [ freq ] = vehicle_frequence (vehicle_params, num_pass, num_round_trip_hr, num_vehicles, time_hr, n_variations)
flow = num_pass;
freq = zeros(n_variations,length(time_hr));
for i = 2:n_variations
    for  j = 1:length(time_hr)
        if (flow(i,j) < vehicle_params.n_pass) && (flow(i,j) > 0)
            freq(i,j) = 1;
        elseif (flow(i,j)> vehicle_params.n_pass) && (flow(i,j) < vehicle_params.n_pass * num_round_trip_hr)
            freq(i,j) = num_round_trip_hr ;
        elseif flow(i,j)>= vehicle_params.n_pass * num_round_trip_hr * num_vehicles(i,j)
            freq(i,j) = num_round_trip_hr * num_vehicles(i,j);
        else
            freq(i,j) = ceil(flow(i,j) / vehicle_params.n_pass);
        end
    end
end
end
