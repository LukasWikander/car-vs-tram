function opt = assignment_cost_function(outputs, ROI_yr)

%ASSIGNMENT_COST_FUNCTION Summary of this function goes here
%   Detailed explanation goes here
%% Evaluate accumulated costs at different time horizons
yr_idx = round(ROI_yr*365);

[opt.cost,opt.mix] = min(outputs.results.costs.cost_grid_mix_day(:,yr_idx));
end

