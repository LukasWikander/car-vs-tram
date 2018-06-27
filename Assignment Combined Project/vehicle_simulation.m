function [ output ] = vehicle_simulation( vehicle_params, general_params, drv_mission, pass_flow )
%VEHICLE_SIMULATION Summary of this function goes here
%   Detailed explanation goes here

output = struct;
output.E_traction_tot_kWh = 0;
output.E_brake_tot_kWh = 0;
output.E_tot_kWh = 0;
output.P_traction_max_kW = 0;
output.P_brake_max_kW = 0;
output.P_max_kW = 0;
output.P_traction_W = {};
output.P_brake_W = {};
output.speed_kmh = {};
output.time = {};
output.t_round_trip = 0;
output.t_charging = 0;

simulated_drive_cycles = [1, 2];

output.req_battery_size_energy_kWh = 0;
output.req_battery_size_chargeTime_kWh = 0;
output.req_battery_size_kWh = 0;
output.req_recharge_energy_kWh = zeros(length(simulated_drive_cycles),1);

%% Ideal vehicle
%------------------------------------------------
% Components and Systems for Electromobility
% - Practical session I
% 
% Script file for running idealvehicle.mdl
% - Ideal powertrain, 100% efficiency
%------------------------------------------------

%------------------------------------------------
% Vehicle parameters
%------------------------------------------------
% Combustion engine parameters  ++++++++++++++++++

vehicle_params.m_kg = vehicle_params.m_kg + vehicle_params.n_pass*general_params.m_pass_kg; % Vehicle weight

%------------------------------------------------
% Environment parameters
%------------------------------------------------
t_charge_full_h = 1/general_params.C_rate;

% Needed by simulation:
Mv = vehicle_params.m_kg;
rw = vehicle_params.rw; % Wheel radius (m)
Cd = vehicle_params.Cd; % Air resistance
Cr = vehicle_params.Cr;	% Roll resistance
Av = vehicle_params.Ad;	% Front area
vmax=vehicle_params.v_max_kmh / 3.6; % Max velocity
Pmax=vehicle_params.P_max_kW * 2000; % Max power

rho_air = general_params.rho_air;
grav = general_params.acc_gravity;
Density = 27.78e6;


% Simulate vehicle
for i = simulated_drive_cycles
	
	if i == 1
		dc = drv_mission.dc;
	elseif i == 2
		dc = drv_mission.dc;
		dc.slope = -1*flipud(dc.slope);
		dc.alt = flipud(dc.alt);
		dc.altinit = dc.alt(1);
		dc.pos = flipud(dc.pos);
	end
	
	%% Dynamic programming to calculate optimal velocity profile
	fprintf('Optimizing velocity profile...\n')
	velocity_optimization_results = velocity_optimization(vehicle_params, general_params, dc);

	t = velocity_optimization_results.t;
	v = velocity_optimization_results.v;

    [cycle, endtime, cycle_name] = form_simulation_cycle(i, drv_mission.dc, t, v);
    
    %% Simulate vehicle
    options = simset('SrcWorkspace','current');
    sim('idealvehicle',[],options);
    
    %% Energy analysis 
    EnergyTraction_tot = Energy.signals(1).values(end);
    EnergyBrake_tot = Energy.signals(4).values(end);
	EnergyStored_kWh = (Energy.signals(4).values - Energy.signals(1).values) /60/60/1000;
    MaxTractionPower = max(abs(Power.signals(3).values));
    MaxBrakingPower = max(abs(Power.signals(5).values));
    tTrip = endtime;
    
    %% Add results to output
    output.E_traction_tot_kWh = output.E_traction_tot_kWh + EnergyTraction_tot /60/60/1000;
    output.E_brake_tot_kWh = output.E_brake_tot_kWh + EnergyBrake_tot /60/60/1000;
    output.P_traction_max_kW = max([output.P_traction_max_kW, MaxTractionPower / 1000]);
    output.P_brake_max_kW = max([output.P_brake_max_kW, MaxBrakingPower / 1000]);
	output.P_traction_W{i} = Power.signals(3).values;
	output.P_brake_W{i} = Power.signals(5).values;
	output.speed_kmh{i} = Power.signals(1).values;
    output.t_round_trip = output.t_round_trip + tTrip;
	
    %% Battery size calculation
	output.req_recharge_energy_kWh(i) = EnergyStored_kWh(1) - EnergyStored_kWh(end);
	% Minimum battery size to meet energy constraints
	req_size_energy_kWh = max(EnergyStored_kWh) - min(EnergyStored_kWh);
	% Minimum battery size to meet charging time constraints
	req_size_chargeTime_kWh = output.req_recharge_energy_kWh(i)/(vehicle_params.t_unload/60/60 /t_charge_full_h);
	
	% To outputs
	output.req_battery_size_energy_kWh = max([output.req_battery_size_energy_kWh req_size_energy_kWh]);
	output.req_battery_size_chargeTime_kWh = max([output.req_battery_size_chargeTime_kWh req_size_chargeTime_kWh]);
	output.req_battery_size_kWh = max([output.req_battery_size_kWh max([req_size_energy_kWh req_size_chargeTime_kWh])]);
	output.time{i} = Power.time;
    
end
% These parameters can only be known when battery size is known
for i = simulated_drive_cycles
	output.t_charging(i) = max([vehicle_params.t_unload output.req_recharge_energy_kWh(i)/output.req_battery_size_kWh*t_charge_full_h *60*60]);
	output.req_charge_power_kW(i) = output.req_recharge_energy_kWh(i)/(output.t_charging(i) /60/60);
end
output.t_charging_round_trip = sum(output.t_charging);

output.P_max_kW = max([output.P_traction_max_kW, output.P_brake_max_kW]);
output.E_tot_kWh = output.E_traction_tot_kWh - output.E_brake_tot_kWh;
end

function [ cycle, endtime, cycle_name ] = form_simulation_cycle ( idx, dc, t, v )
%------------------------------------------------
% Convert topology to drive cycle
%------------------------------------------------
Drive_cycles = cellstr(['A to B'; 'B to A']);
M = [0.5*eye(length(t)-1) zeros(length(t)-1,1)] + [zeros(length(t)-1,1) 0.5*eye(length(t)-1)]; % Mean adjacent value matrix
mt = M*t;

cycle = zeros(length(mt),5);
switch idx
	case 1
		cycle(:,1) = mt;
		cycle(:,2) = v;
		cycle(:,3) = cumsum(v.*diff(t));
		cycle(:,4) = interp1(dc.s,dc.alt,cycle(:,3),'pchip');
		cycle(:,5) = interp1(dc.s,dc.slope,cycle(:,3),'pchip');
	case 2
		cycle(:,1) = mt;
		cycle(:,2) = v;
		cycle(:,3) = flipud(cumsum(v.*diff(t)));
		cycle(:,4) = flipud(interp1(dc.s,dc.alt,flipud(cycle(:,3)),'pchip'));
		cycle(:,5) = -1*flipud(interp1(dc.s,dc.slope,flipud(cycle(:,3)),'pchip'));
		
end
cycle_name = Drive_cycles(idx);
endtime = max(cycle(:,1));
end

