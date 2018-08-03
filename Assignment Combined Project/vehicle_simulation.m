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

%% Electrical vehicle
%------------------------------------------------
% Environment parameters
%------------------------------------------------
t_charge_full_h = 1/general_params.C_rate;

%% ------------------------------------------------
% Vehicle parameters
%------------------------------------------------

vehicle_params.m_kg = vehicle_params.m_kg + vehicle_params.n_pass*general_params.m_pass_kg; % Vehicle weight [kg]
Mv = vehicle_params.m_kg; 
Maxle = Mv/2;
rw = vehicle_params.rw; % Wheel radius [m]
Cd = vehicle_params.Cd; % Air resistance [-]
Cr = vehicle_params.Cr;	% Roll resistance [-]
Av = vehicle_params.Ad;	% Front area [m^2]
vmax = vehicle_params.v_max_kmh / 3.6; % Max velocity [m/s]
Paux = 0; %Auxiliary power offered to the battery [W]
Acc50 = 15;  % Acceleration time to 50 km/h in seconds

%% ------------------------------------------------
% Performance requirements
%------------------------------------------------

rho_air = general_params.rho_air; % Air density [kg/m3]
grav = general_params.acc_gravity; % Acceleration of gravity [m/s2]
Density = 27.78e6; %Fuel energy density [Ws/liter] (gasoline selected)

Max_start_slope = max(drv_mission.dc.slope)*100; % [%]
Tw_slope = rw*Mv*grav*sin(atan(Max_start_slope/100)); % Assumed at 60 km/h
Tw_roll = Mv*grav*Cr*rw;
Tw_drag = rw*0.5*Av*Cd*rho_air*vmax^2;
Tw_acc = Maxle*(50/3.6/Acc50)*rw;
Max_curb_hight = 0.15;  % [m]
hc=Max_curb_hight;  
Tw_curb = rw*Maxle*grav*sin(acos(-(hc-rw)/rw)); % At zero speed
%% Simulate vehicle
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
	velocity_optimization_results = velocity_optimization(vehicle_params, general_params, dc);
	t = velocity_optimization_results.t;
	v = velocity_optimization_results.v;
    
    [cycle, endtime, cycle_name] = form_simulation_cycle(i, drv_mission.dc, t, v);
    
    %% Defining maximum required power
    
    for index = 1:length(v)
        if v(index) <= 50/3.6
            Tmax(index)= Tw_roll+Tw_slope+Tw_acc+rw*0.5*Av*Cd*rho_air*v(index)^2; %Full capabilities up to 50 [km/h]
            if Tmax(1)<Tw_curb
            Tmax(1) = Tw_curb; % Needed to start
        end
        elseif v(index) > 50/3.6 && v(index) <= 80/3.6
            Tmax(index) = Tw_roll+(Tw_acc+Tw_slope)*(1-(v(index)-50/3.6)/(30/3.6))+rw*0.5*Av*Cd*rho_air*v(index)^2;
        % Acceleration and slope climbing fades off until 80 [km/h];
        elseif v(index) > 80/3.6
            Tmax(index) = Tw_roll+rw*0.5*Av*Cd*rho_air*v(index)^2; % Above 80 [km/h] only able to keep speed at flat road
        end
        Tdrag(index) = rw*0.5*Av*Cd*rho_air*v(index)^2;
    end
    Pmax = v'.*Tmax/rw;     %Max required power during one drive cycle [W]

    %% *** Transmission settings **********************************************
    utvx_min = 10;  % "Highest gear". 12.3;  
    EtaGEAR = 0.97; % Simplest possible efficiency model of the transmission
    Number_of_gears = 1; 
    DCT=1;
    Utvx_vect = zeros(1,Number_of_gears+1);
    Utvx_vect(1,1) = utvx_min;
    Utvx_vect(1,length(Utvx_vect(1,:))) = inf;
    
    %% *** Electric machine settings  ***************************************
    fwr = 2.8; % Field weakening ratio, i.e. ratio between max speed and base speed
    wem_min = 0;
    wem_vmax = vmax*utvx_min/rw;    % EM Speed at max vehicle speed [r/s]
    wem_max = 1.5*wem_vmax;         % EM max speed [r/s]
    Pem_max = max(Pmax); %EM max power required [W] 
    Tem_max = Pem_max/(wem_max/fwr); % Peak continuous torque [Nm]
    [PtoTem,EtaEM,Tem,Wem,Tem_lim,EnergyConsEM] = CreateEMmap(Pem_max,wem_max,Tem_max);
    utvx_max = Tmax(1)/max(Tem_lim); % "Lowest gear - to climb a curb".
    
    %% *** Power electronics settings ***************************************
    EtaPE = 0.97; % PE efficiency
    
    %% *** Battery settings *************************************************
    % The battery model is a constant voltage in series with a resistance. The
    % electric variables voltage and current is recalculated into terminal power
    % and efficiencies. This is done in the call to "CreateBATTmap".
    % 
    Wbatt = velocity_optimization_results.Ereq_batt_kWh; %Battery energy [Ws]
    [EtaBATT,Pbatt,Mbatt,Pbatt_max_new,Wbatt_new]=CreateBATTmap(Pem_max,Wbatt);
    Wbatt=Wbatt_new;
%     velocity_optimization_results.Ereq_batt_kWh = Wbatt;
    SOC_batt_ref_value = 20; % target value for SOC control
    SOC_batt_start_value = 90; % Start value of SOC
    SOC_tract_max = 95; % Max SOC level, no negative EM power allowed above.
    SOC_tract_min = 15; % Min SOC level, no positive EM power allowed below.

    %% Simulate vehicle
    options = simset('SrcWorkspace','current');
    sim('EVmodel',[],options);
    
    %% Energy analysis 
    EnergyTraction_tot = Energy.signals(1).values(end);
    EnergyBrake_tot = Energy.signals(2).values(end);
	EnergyStored_kWh = (Energy.signals(2).values - Energy.signals(1).values) /60/60/1000;
    MaxTractionPower = max(abs(Power.signals(3).values)*1000);
    MaxBrakingPower = max(abs(Power.signals(5).values)*1000);
    tTrip = endtime;
    
    %% Add results to output
    output.E_traction_tot_kWh = output.E_traction_tot_kWh + EnergyTraction_tot /60/60/1000;
    output.E_brake_tot_kWh = output.E_brake_tot_kWh + EnergyBrake_tot /60/60/1000;
    output.P_traction_max_kW = max([output.P_traction_max_kW, MaxTractionPower / 1000]);
    output.P_brake_max_kW = max([output.P_brake_max_kW, MaxBrakingPower / 1000]);
	output.P_traction_W{i} = Power.signals(3).values*1000;
	output.P_brake_W{i} = Power.signals(5).values*1000;
	output.speed_kmh{i} = Power.signals(1).values*1000;
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
	%output.req_battery_size_kWh = max([output.req_battery_size_kWh max([req_size_energy_kWh req_size_chargeTime_kWh])]);
	output.req_battery_size_kWh = output.req_battery_size_energy_kWh;
	output.time{i} = Power.time;
    
    
 end
% These parameters can only be known when battery size is known
for i = simulated_drive_cycles
	output.t_charging(i) = vehicle_params.t_unload;%max([vehicle_params.t_unload output.req_recharge_energy_kWh(i)/output.req_battery_size_kWh*t_charge_full_h *60*60]);
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

