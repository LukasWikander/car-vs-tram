function y=costFunction(x)

% Initialize for two objectives 
y = 0;

 
% Compute the cost function
y = 1/3*((x(1)+2).^2 - 10)+2/3*((x(2)-2).^2 + 10);

end