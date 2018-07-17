function sensitivity = sensitivity_analysis(general_params,tram_params,car_params)

disp('Running sensitivity analysis ...')
pause(1)

%% Settings
sz_h = 0.01;						% Percentage to vary assumption

%% Sensitivity analysis
for ii = 1:3
	% Check the three assumptions structs separately
	switch ii
		case 1
			sens_fld_name = 'general_params';
			investigated_params = general_params;
			assignment_fcn = @(inv_params)(init_assignment(inv_params,tram_params,car_params,false));
		case 2
			sens_fld_name = 'tram_params';
			investigated_params = tram_params;
			assignment_fcn = @(inv_params)(init_assignment(general_params,inv_params,car_params,false));
		case 3
			sens_fld_name = 'car_params';
			investigated_params = car_params;
			assignment_fcn = @(inv_params)(init_assignment(general_params,tram_params,inv_params,false));
	end
	
	% Loop over all fields in struct
	fnames = fieldnames(investigated_params);
	for field_no = 1:length(fnames)
		if ischar(investigated_params.(fnames{field_no}))
			sensitivity.(sens_fld_name).(fnames{field_no}) = NaN;
			continue
		end

		disp(['Checking assumption on ''' fnames{field_no} ''' ...'])

		% Central difference approximation
		h = sz_h*investigated_params.(fnames{field_no});
		investigated_params_p = investigated_params;
		investigated_params_p.(fnames{field_no}) = investigated_params_p.(fnames{field_no}) + h;
		investigated_params_n = investigated_params;
		investigated_params_n.(fnames{field_no}) = investigated_params_n.(fnames{field_no}) - h;

		output_p = assignment_fcn(investigated_params_p);
		output_n = assignment_fcn(investigated_params_n);

		[f_p(1), f_p(2)] = assignment_cost_function(output_p);
		[f_n(1), f_n(2)] = assignment_cost_function(output_n);

		sensitivity.(sens_fld_name).(fnames{field_no}) = (f_p - f_n)./(2.*h);
	end
end

end

