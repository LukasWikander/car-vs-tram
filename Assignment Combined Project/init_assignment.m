clear all
close all
clc

%% Load variables, initialise
drv_mission  = load ('dc_AtoB.mat');
pass_flow    = load ('passengers.mat');

l = diff(drv_mission.dc.s);
M = [0.5*eye(length(drv_mission.dc.slope)-1) zeros(length(drv_mission.dc.slope)-1,1)] ...
	+ [zeros(length(drv_mission.dc.slope)-1,1) 0.5*eye(length(drv_mission.dc.slope)-1)]; % Mean adjacent value matrix

a = M*drv_mission.dc.slope; 
drv_mission.dc.pos = cumsum([0; l.*cos(a)]); % Flight distance [m]
drv_mission.dc.alt = cumsum([drv_mission.dc.altinit; l.*sin(a)]);

%% Assignment given parameters
general_params = struct;
general_params.m_pass_kg = 80; % Passenger weight [kg]
general_params.c_E_kWh = 1; % Cost of energy [SEK/kWh]
general_params.v_max_kmh = 70; % Maximum allowed velocity [km/h]
general_params.c_batt_kWh = 1000; % Cost of battery [SEK/kWh]
general_params.e_batt_kWh = 0.1; % Energy density of battery [kWh/kg]
general_params.c_motor_kW = 70+50; % Cost of motor + cost of power electronics [SEK/kW]
general_params.C_rate = 1.5; % Charge C rate [A/Ah]
general_params.c_station_kW = 2000; % Charge station cost [SEK/kW]
general_params.c_stop = 3.3*1e6; % Tram or car stop (station) [SEK/location]
general_params.n_variations = 10; % Number of tram variations (grid size)

%% Assumed additional parameters
general_params.V_batt = 400;	% Battery pack voltage [V]
general_params.l_trip = drv_mission.dc.s(end); % Length of trip [m]
general_params.l_round_trip = 2*drv_mission.dc.s(end); % Length of round trip [m]
general_params.c_tram_track_km = 112e6; % Cost of track [SEK/km]
general_params.c_tram_track_mnt_yr = (368+172)/5/249*10*1e6; % Cost of track yearly when in use [SEK/km/year]
general_params.l_life_tram_yr = 40; % Lifetime of tram [year]
general_params.l_life_car_km = 160000; % Expected mileage of car [km]
general_params.t_zerotomax = 20; % Acceleration time for constructed drive cycle [s]

general_params.rho_air = 1.1839;	% Air density [kg/m3]
general_params.acc_gravity = 9.81;	% Acceleration of gravity [m/s2]

%% Tram vehicle parameters
tram_params = struct;
tram_params.m_kg = 36800; % Tram weight [kg]
tram_params.n_pass = 85 + 117; % Seated + standing passenger capacity 
tram_params.Cd = 1.8; % Tram drag coefficient
tram_params.Ad = 3.32*(1.435+0.4); % Tram frontal area
tram_params.Cr = 0.001; % Tram wheel rolling resistance
tram_params.rw = 0.2;	% Tram wheel radius
tram_params.t_unload = 2 * 60; % Time at stops [s]
tram_params.c_purchase = 1.08*37600000/18*11.75 + 3500 + 48000; % Purchase cost of tram + gear box + AD components [SEK]
tram_params.c_maintenance = 1.3; % Maintenance cost of tram + track per km driven [SEK/km]
tram_params.c_parking = (67.5/7)*1e6; % Parking cost (tram depot) [SEK/vehicle] (Not realistic, probably ~ 40 + 4*n_trams)
tram_params.v_max_kmh = 70; % Maximum allowed velocity [km/h] (see assignment)
tram_params.P_max_kW = inf; % Maximum power [kW]

% Dynamic programming cost function parameters:
tram_params.accpenalty = 0.25;
tram_params.traveltimepenalty = 0.25*0.058;
tram_params.energypenalty = 0.1;

%% Car vehicle parameters
car_params = struct;
car_params.m_kg = 2500; % Car weight [kg]
car_params.n_pass = 10;	% Passenger capacity
car_params.Cd = 0.5; % Car drag coefficent
car_params.Ad = 1.79*1.9; % Car frontal area
car_params.rw = 0.37;	% Car wheel radius
car_params.Cr = 0.02;	% Car rolling resistance
car_params.t_unload = 5*60;   % Time at stops [s]
car_params.c_purchase = 300000+3500+48000; % Purchase cost of car (base + gear box + AD components) [SEK]
car_params.c_maintenance = 0.277; % Maintenance cost of car per km driven [SEK/km]
car_params.c_parking = 0; % Parking cost [SEK/vehicle] (TODO)
car_params.v_max_kmh = 70; % Maximum allowed velocity [km/h] (see assignment)
car_params.P_max_kW = inf; % Maximum power [kW]

% Dynamic programming cost function parameters:
car_params.accpenalty = 0.25;
car_params.traveltimepenalty = 0.058;
car_params.energypenalty = 0.1;

% NOTE: Increased battery size (compared to trip traction energy) to 
% decrease charging time per trip

%% Vehicle simulation
% Full tram
fprintf(' --- Full tram simulation --- \n')
tram_full_sim_output = vehicle_simulation(tram_params, general_params, drv_mission, pass_flow);

% Empty tram
fprintf(' --- Empty tram simulation --- \n')
tram_empty_params = tram_params;
tram_empty_params.n_pass = 0;
tram_empty_sim_output = vehicle_simulation(tram_empty_params, general_params, drv_mission, pass_flow);

% Full car
fprintf(' --- Full car simulation --- \n')
car_full_sim_output = vehicle_simulation(car_params, general_params, drv_mission, pass_flow);

% Empty car
fprintf(' --- Empty car simulation --- \n')
car_empty_params = car_params;
car_empty_params.n_pass = 0;
car_empty_sim_output = vehicle_simulation(car_empty_params, general_params, drv_mission, pass_flow);

%% Fleet size estimation
tram_params.t_round_trip = tram_full_sim_output.t_round_trip;
tram_params.t_charging_round_trip = tram_full_sim_output.t_charging_round_trip;
tram_params.E_battery_size_kWh = tram_full_sim_output.req_battery_size_kWh;
tram_params.E_round_trip_kWh = tram_full_sim_output.E_tot_kWh;
car_params.t_round_trip = car_full_sim_output.t_round_trip;
car_params.t_charging_round_trip = car_full_sim_output.t_charging_round_trip;
car_params.E_battery_size_kWh = car_full_sim_output.req_battery_size_kWh;
car_params.E_round_trip_kWh = car_full_sim_output.E_tot_kWh;
fleet_info_output = fleet_size_estimation(tram_params, car_params, general_params, drv_mission, pass_flow, general_params.n_variations);

%% Cost estimation
% TODO: Use empty vehicle simulation to get more accurate costs
cost_estimation_output = cost_estimation(tram_params, car_params, general_params, tram_full_sim_output, car_full_sim_output, fleet_info_output);

%% Plots
% TODO: Plots
plot_assignment(tram_params, car_params, general_params, drv_mission, pass_flow, tram_full_sim_output, car_full_sim_output, fleet_info_output, cost_estimation_output);

fprintf('Done!\n');