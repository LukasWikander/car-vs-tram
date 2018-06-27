function res = velocity_profile(general_params, dc)

%% Calculate simple velocity profile for benchmarking
%l = diff(dc.s);
%M = [0.5*eye(length(dc.slope)-1) zeros(length(dc.slope)-1,1)] + [zeros(length(dc.slope)-1,1) 0.5*eye(length(dc.slope)-1)]; % Mean adjacent value matrix
% a = M*dc.slope; 
% dc.pos = cumsum([0; l.*cos(a)]); % Flight distance [m]
% dc.alt = cumsum([dc.altinit; l.*sin(a)]);
amax = general_params.v_max_kmh/3.6/general_params.t_zerotomax; % Maximum allowed acceleration [m/s^2]

% Generate simple accelerate-vmax-decelerate velocity profile
t0 = 0;
t1 = general_params.v_max_kmh/3.6/amax;
t2 = (dc.s(end) + (general_params.v_max_kmh/3.6)^2/amax)/(general_params.v_max_kmh/3.6);
t3 = t2 + general_params.v_max_kmh/3.6/amax;
t = (0:t3)';
v = amax*t;
v(v>general_params.v_max_kmh/3.6) = 0;
v = v + flipud(v);
v(v==0) = general_params.v_max_kmh/3.6;
v(1) = 0; 
v(end) = 0;

res.v=v;
res.t=t;

end

