function [opt_upfront,opt_5yrs,opt_10yrs,opt_15yrs,opt_20yrs] = assignment_cost_function(outputs)

%ASSIGNMENT_COST_FUNCTION Summary of this function goes here
%   Detailed explanation goes here
%% Evaluate accumulated costs at different time horizons
yr5_idx = 5*365;
yr10_idx = 10*365;
yr15_idx = 15*365;
yr20_idx = 20*365;

opt_upfront.cost = outputs.results.costs.min_cost_purchase;
opt_upfront.mix = outputs.results.costs.min_upfront_alt;

[opt_5yrs.cost,opt_5yrs.mix] = min(outputs.results.costs.cost_grid_mix_day(:,yr5_idx));
[opt_10yrs.cost,opt_10yrs.mix] = min(outputs.results.costs.cost_grid_mix_day(:,yr10_idx));
[opt_15yrs.cost,opt_15yrs.mix] = min(outputs.results.costs.cost_grid_mix_day(:,yr15_idx));
[opt_20yrs.cost,opt_20yrs.mix] = min(outputs.results.costs.cost_grid_mix_day(:,yr20_idx));
end

