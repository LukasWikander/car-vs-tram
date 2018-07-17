function [cost_upfront,upfront_alt] = assignment_cost_function(outputs)
%ASSIGNMENT_COST_FUNCTION Summary of this function goes here
%   Detailed explanation goes here
%% TODO: Add more stuff to this cost function!
cost_upfront = outputs.results.costs.min_cost_purchase;
upfront_alt = outputs.results.costs.min_upfront_alt;
end

