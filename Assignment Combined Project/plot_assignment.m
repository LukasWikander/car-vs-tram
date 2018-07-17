function [ output ] = plot_assignment( tram_params, car_params, general_params, drv_mission, pass_flow, tram_simulation_results, car_simulation_results, fleet_info_output, cost_estimation_output)
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here
%% Power profiles
figure('Name','Power profile tram')
subplot(211)
plot(tram_simulation_results.time{1}, (tram_simulation_results.P_traction_W{1}+tram_simulation_results.P_brake_W{1})/1000)
title('A to B')
ylabel('Power [kW]')
xlabel('Time [s]')
grid on

subplot(212)
plot(tram_simulation_results.time{2}, (tram_simulation_results.P_traction_W{2}+tram_simulation_results.P_brake_W{2})/1000)
title('B to A')
ylabel('Power [kW]')
xlabel('Time [s]')
grid on

%
figure('Name','Power profile car')
subplot(211)
plot(car_simulation_results.time{1}, (car_simulation_results.P_traction_W{1}+car_simulation_results.P_brake_W{1})/1000)
title('A to B')
ylabel('Power [kW]')
xlabel('Time [s]')
grid on

subplot(212)
plot(car_simulation_results.time{2}, (car_simulation_results.P_traction_W{2}+car_simulation_results.P_brake_W{2})/1000)
title('B to A')
ylabel('Power [kW]')
xlabel('Time [s]')
grid on

%% Speed profiles
figure('Name','Speed profile tram')
subplot(211)
plot(tram_simulation_results.time{1}, tram_simulation_results.speed_kmh{1})
title('A to B')
ylabel('Speed [km/h]')
xlabel('Time [s]')
grid on

subplot(212)
plot(tram_simulation_results.time{2}, tram_simulation_results.speed_kmh{2})
title('B to A')
ylabel('Speed [km/h]')
xlabel('Time [s]')
grid on

%
figure('Name','Speed profile car')
subplot(211)
plot(car_simulation_results.time{1}, car_simulation_results.speed_kmh{1})
title('A to B')
ylabel('Speed [km/h]')
xlabel('Time [s]')
grid on

subplot(212)
plot(car_simulation_results.time{2}, car_simulation_results.speed_kmh{2})
title('B to A')
ylabel('Speed [km/h]')
xlabel('Time [s]')
grid on


%% Lifetime costs
dy = 0:365*80;
daily_cost_grid = cost_estimation_output.fleet_cost_dy * dy;
dy_size = size(dy);
n_size = size(cost_estimation_output.fleet_cost_dy);

tram_purchase_grid = ceil(ones(n_size) * dy / 365 ./ ((general_params.l_life_tram_yr * ones(n_size)) * ones(dy_size)));
car_purchase_grid = ceil(ones(n_size) * dy /365 ./ (cost_estimation_output.l_life_car_yr * ones(dy_size)));
car_purchase_grid(isnan(car_purchase_grid)) = 0; % When using only trams, no cars are purchased

Z = ((cost_estimation_output.trams_cost_purchase' * ones(dy_size)) .* tram_purchase_grid ...
	+ (cost_estimation_output.cars_cost_purchase' * ones(dy_size)) .* car_purchase_grid ...
	+ daily_cost_grid) *1e-6;

% Minimum cost
[ZM,I] = min(Z);
YM = nan(size(I));
for ii = 1:length(I)
	YM(ii) = fleet_info_output.num_trams(I(ii));
end

[XG,YG] = meshgrid(dy/365, fleet_info_output.num_trams);
figure('Name','Accumulated costs')
surf(XG, YG, Z, 'EdgeColor', 'None')
hold on
hnd=plot3(dy/365,YM,ZM,'r','LineWidth',1.5);
legend(hnd,'Cost minimizing mix','Location','best')
xlabel('Time [years]')
ylabel('Number of trams in mix')
zlabel('Cost [MSEK]')

% Same but contour plot
figure('Name','Isocost levels')
[Cp,hp] = contour(XG, YG, Z, 20, 'ShowText', 'on');
xlabel('Time [years]')
ylabel('Number of trams in mix')
clabel(Cp,hp,'LabelSpacing',1000)
title('Isocost levels [MSEK]')
grid on
hold on
hnd=plot(dy/365,YM,'r','LineWidth',1.5);
legend(hnd,'Cost minimizing mix','Location','northwest')


%% Fleet mix
figure('Name','Fleet mix')
plot(fleet_info_output.num_trams,fleet_info_output.num_cars,'rx')
xlabel('Number of trams')
ylabel('Number of cars')
grid on

%% Energy stored in fleet
[~,TG] = meshgrid(pass_flow.x, fleet_info_output.num_trams);
[HG,CG] = meshgrid(pass_flow.x, fleet_info_output.num_cars);

figure('Name','Energy stored in fleet')
surf(HG,TG,(fleet_info_output.car_energy_remaining_grid+fleet_info_output.tram_energy_remaining_grid));
xlabel('Hour of day')
xlim([4 24])
ylabel('Number of trams')
zlabel('Energy remaining [kWh]')
ax = gca;
ax.XAxis.TickValues = pass_flow.x;
ax.YAxis.TickValues = fleet_info_output.num_trams;
ax.YAxis.Direction = 'reverse';
title('Total energy stored in fleet')

%% Average energy content of vehicle
figure('Name','Energy stored on average')
subplot(121)
surf(HG,TG,100*(fleet_info_output.tram_energy_remaining_grid./fleet_info_output.num_trams')./tram_params.E_battery_size_kWh)
ax = gca;
ax.XAxis.TickValues = pass_flow.x;
ax.YAxis.TickValues = fleet_info_output.num_trams;
ax.YAxis.Direction = 'reverse';
ax.ZAxis.TickValues = 0:10:100;
[az,el] = view;
view(az+60,el);
xlabel('Hour of day')
xlim([4 24])
ylabel('Number of trams')
zlabel('State of charge [%]')
title('Average SoC per tram')

subplot(122)
surf(HG,CG,100*(fleet_info_output.car_energy_remaining_grid./fleet_info_output.num_cars')./car_params.E_battery_size_kWh)
ax = gca;
ax.XAxis.TickValues = pass_flow.x;
ax.YAxis.TickValues = fliplr(fleet_info_output.num_cars);
ax.YAxis.Direction = 'reverse';
ax.ZAxis.TickValues = 0:10:100;
[az,el] = view;
view(az+60,el);
xlabel('Hour of day')
xlim([4 24])
ylabel('Number of cars')
zlabel('State of charge [%]')
title('Average SoC per car')


%% Cost of charging stations
figure('Name','Cost of charging stations')
plot(fleet_info_output.num_trams, cost_estimation_output.car_chargers_cost_purchase ...
	+ cost_estimation_output.tram_chargers_cost_purchase,'LineWidth',1.5)
hold on
plot(fleet_info_output.num_trams, cost_estimation_output.tram_chargers_cost_purchase,'--','LineWidth',1.5)
plot(fleet_info_output.num_trams, cost_estimation_output.car_chargers_cost_purchase,'--','LineWidth',1.5)
legend({'Cost of charging stations','Cost of tram charging stations','Cost of car charging stations'})
ax = gca;
ax.XAxis.TickValues = fleet_info_output.num_trams;
xlabel('Number of trams')
ylabel('Cost [SEK]')
grid on


end

