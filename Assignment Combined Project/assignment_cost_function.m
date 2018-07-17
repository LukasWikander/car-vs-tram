function [cost_upfront] = assignment_cost_function(outputs)
%ASSIGNMENT_COST_FUNCTION Summary of this function goes here
%   Detailed explanation goes here
%% TODO: Add more stuff to this cost function!
cost_upfront = outputs.results.costs.min_cost_purchase;
end

