% This script calculates the number of vehicles in the fleet
% for transporting passengers from a City to a residential and industrial
% area. Two different vehicle types are assumed: tram and 4-10 seat
% car.

close all
clear 
clc

fprintf('Home Assignment Swedish Electromobility Centre\n')
fprintf('Summer School 2018\n')
fprintf('Fleet size\n\n')

% General data

avg_speed_tram          = 50 * 5 / 18;             % Average speed of tram in [m/s]
avg_speed_car           = 60 * 5 / 18;             % Average speed of car in [m/s]
pass_per_tram           = 100;                     % Maximum capacity of each tram
pass_per_car            = 5;                       % Passenger capacity of the car
deborad_tram            = 2;                       % Deboarding time at each stop [min]
deborad_car             = 0.5;                     % Deboarding time at each stop [min]
n_tram                  = 10;                      % Number of variations

% Driving mission
% The driving mission is based on slope [rad] with respect to distance

drv_mission  = load ('dc_AtoB.mat');
slope        = drv_mission.dc.slope;
distance     = drv_mission.dc.s;

height = 12; width = 16;
top = 1; bottom = 1.5; left = 2; right = 1;
figure_configuration_code

figure(1)
plot(distance/1e3,slope,'b')
xlabel('Distance [km]')
ylabel('Slope [rad]')

% Passenger flow
% The number of passengers travelling from A to B during different time of
% the day

pass_flow    = load ('passengers.mat');
time_hr      = pass_flow.x;
from_A2B     = pass_flow.yA;
from_B2A     = pass_flow.yB;

width = 20;
figure_configuration_code

%% Fleet size

mean_flow     = sum (from_A2B) / (max(time_hr) - min(time_hr));
mean_flow     = floor(mean_flow); 

figure(2)
subplot(1,2,1)
bar(time_hr,from_A2B,'b')
hold on
plot(time_hr,ones(size(time_hr)) * mean_flow,'g')
hold off
ylim([0 1900])
xlabel('Time of the day [24 hrs]')
ylabel('Number of passengers from A to B')

subplot(1,2,2)
bar(time_hr,from_B2A,'b')
hold on
plot(time_hr,ones(size(time_hr)) * mean_flow,'g')
hold off
ylim([0 1900])
xlabel('Time of the day [24 hrs]')
ylabel('Number of passengers from B to A')

% One way trip length [min]
time_per_trip_tram      = max( distance) / avg_speed_tram / 60; 
% Round trip length [min]
tt_per_round_trip_tram  = 2 * time_per_trip_tram + deborad_tram;
% Number of round trips per hour each tram
num_round_trip_hr_tram  = floor (60 / tt_per_round_trip_tram);
% Maximum number of trams
max_flow_hr             = max(max(from_A2B),max(from_B2A));
max_num_trams           = floor ( max_flow_hr / pass_per_tram / num_round_trip_hr_tram );
% Number of trams
num_trams               = linspace(0,max_num_trams,n_tram);
% Flow capacity of tram line
flow_cap_hr_tram        = pass_per_tram * num_round_trip_hr_tram .* num_trams;

% One way trip length [min]
time_per_trip_car       = max( distance) / avg_speed_car / 60;  
% Round trip length [min]
tt_per_round_trip_car   = 2 * time_per_trip_car + deborad_car;
% Number of round trips per hour each car
num_round_trip_hr_car   = floor ( 60 / tt_per_round_trip_car);   
% Number of cars in fleet
num_car                 = floor (( max(from_A2B) - flow_cap_hr_tram) / pass_per_car / num_round_trip_hr_car );

% Total flow capacity
flow_cap_hr_total       = flow_cap_hr_tram + num_car * pass_per_car * num_round_trip_hr_car;

figure(3)
scatter(num_trams, num_car, 'o','b')
xlabel('Number of trams in the fleet')
ylabel('Number of cars in the fleet')

fprintf('Fleet size\n')
fprintf('Fleet with only trams\n')
fprintf('Number of trams = %.0f \n',max_num_trams)
fprintf('Fleet with only cars\n')
fprintf('Number of cars = %.0f \n',max(num_car))

%% Fleet planning

num_trams = meshgrid(num_trams, time_hr);
num_trams = transpose(num_trams);

from_A2B = transpose(from_A2B);
from_A2B  = repmat(from_A2B, n_tram, 1);

from_B2A = transpose(from_B2A);
from_B2A  = repmat(from_B2A, n_tram, 1);

% Tram frequency from A to B
tram_freq_A2B = zeros(n_tram,length(time_hr));

for i = 2:n_tram
    for  j = 1:length(time_hr)
        if (from_A2B(i,j) < pass_per_tram) && (from_A2B(i,j) > 0)
            tram_freq_A2B(i,j) = 1;
        elseif (from_A2B(i,j)> pass_per_tram) && (from_A2B(i,j) < pass_per_tram * num_round_trip_hr_tram)
            tram_freq_A2B(i,j) = num_round_trip_hr_tram ;
        elseif from_A2B(i,j)>= pass_per_tram * num_round_trip_hr_tram * num_trams(i,j)
            tram_freq_A2B(i,j) = num_round_trip_hr_tram * num_trams(i,j);
        else
            tram_freq_A2B(i,j) = ceil (from_A2B(i,j) / pass_per_tram);
        end
    end
end

% Tram frequency from B to A
tram_freq_B2A = zeros(n_tram,length(time_hr));

for i = 2:n_tram
    for  j = 1:length(time_hr)
        if (from_B2A(i,j) < pass_per_tram) && (from_B2A(i,j) > 0)
            tram_freq_B2A(i,j) = 1;
        elseif (from_B2A(i,j)> pass_per_tram) && (from_B2A(i,j) < pass_per_tram * num_round_trip_hr_tram)
            tram_freq_B2A(i,j) = num_round_trip_hr_tram ;
        elseif from_B2A(i,j)>= pass_per_tram * num_round_trip_hr_tram * num_trams(i,j)
            tram_freq_B2A(i,j) = num_round_trip_hr_tram * num_trams(i,j);
        else
            tram_freq_B2A(i,j) = ceil (from_B2A(i,j) / pass_per_tram);
        end
    end
end

% Tram frequency including tram returning empty
tram_freq = max(tram_freq_A2B, tram_freq_B2A);
utilization_tram = round ( 100 * tram_freq ./ num_trams /num_round_trip_hr_tram );
empty_tram_A2B = tram_freq - tram_freq_A2B;
empty_tram_B2A = tram_freq - tram_freq_B2A;

% tram frequency
figure(4)
hold on
n = 10;
contourf(time_hr,num_trams(:,1),tram_freq,n,'LineColor','none');
hold off
caxis([1,20])
colormap(jet(n))
c = colorbar;
c.Label.String = 'Frequency of tram per hour';
xlim([min(time_hr),max(time_hr)])
ylim([1,max(num_trams(:,1))])
xlabel('Hour of the day')
ylabel('Number of trams')

% tram utilization
figure(5)
hold on
n = 10;
contourf(time_hr,num_trams(:,1),utilization_tram,n,'LineColor','none');
hold off
caxis([0,100])
colormap(jet(n))
c = colorbar;
c.Label.String = 'Utilization [%]';
xlim([min(time_hr),max(time_hr)])
ylim([1,max(num_trams(:,1))])
xlabel('Hour of the day')
ylabel('Number of trams')

% Average utilization
avg_utilization_tram = round (sum(utilization_tram, 2)/length(time_hr));
figure(6)
scatter(num_trams(:,1), avg_utilization_tram,'b')
xlabel('Number of trams')
ylabel('Average utilization [%]')

% Empty trams
figure(7)
hold on
n = 10;
contourf(time_hr,num_trams(:,1),empty_tram_A2B,n,'LineColor','none');
hold off
caxis([0,max(max(empty_tram_A2B))])
colormap(jet(n))
c = colorbar;
c.Label.String = 'Empty trams from A to B';
xlim([min(time_hr),max(time_hr)])
ylim([1,max(num_trams(:,1))])
xlabel('Hour of the day')
ylabel('Number of trams')

figure(8)
hold on
n = 10;
contourf(time_hr,num_trams(:,1),empty_tram_B2A,n,'LineColor','none');
hold off
caxis([0,max(max(empty_tram_B2A))])
colormap(jet(n))
c = colorbar;
c.Label.String = 'Empty trams from B to A';
xlim([min(time_hr),max(time_hr)])
ylim([1,max(num_trams(:,1))])
xlabel('Hour of the day')
ylabel('Number of trams')
