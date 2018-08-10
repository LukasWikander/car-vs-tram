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
		elseif strcmp(fnames{field_no},'t_zerotomax') || strcmp(fnames{field_no},'P_max_kW') ...
				|| strcmp(fnames{field_no},'n_variations') || strcmp(fnames{field_no},'passenger_flow_multiplier')
			sensitivity.(sens_fld_name).(fnames{field_no}) = NaN;
			continue
		end

		disp(['Checking assumption on ''' fnames{field_no} ''' ...'])

		% Central difference approximation
		h = get_step_size(sz_h,investigated_params,fnames{field_no});
		
		if ~strcmp(fnames{field_no}, 'ROI_horizon_yr')
			investigated_params_p = investigated_params;
			investigated_params_p.(fnames{field_no}) = investigated_params_p.(fnames{field_no}) + h;
			investigated_params_n = investigated_params;
			investigated_params_n.(fnames{field_no}) = investigated_params_n.(fnames{field_no}) - h;

			output_p = assignment_fcn(investigated_params_p);
			output_n = assignment_fcn(investigated_params_n);

			c_p = assignment_cost_function(output_p, general_params.ROI_horizon_yr);
			c_n = assignment_cost_function(output_n, general_params.ROI_horizon_yr);

			sensitivity.(sens_fld_name).(fnames{field_no}) = [(c_p.cost - c_n.cost)./(2.*h); ...
														(c_p.mix - c_n.mix)./(2.*h)];
		else
			output = assignment_fcn(investigated_params);
			
			c_p = assignment_cost_function(output, general_params.ROI_horizon_yr + h);
			c_n = assignment_cost_function(output, general_params.ROI_horizon_yr - h);
			
			sensitivity.(sens_fld_name).(fnames{field_no}) = [(c_p.cost - c_n.cost)./(2.*h); ...
														(c_p.mix - c_n.mix)./(2.*h)];
		end
	end
end

end

function h = get_step_size(pct_h,params,fieldname)
	% If the field is expected to be an integer, return the minimal step size of
	% 1, otherwise return the specified per cent based step size
	switch fieldname
		case 'n_pass'
			h = 1;
		case 'l_life_tram_yr'
			h = 1;
		case 'n_variations'
			h = 1;
		case 'ROI_horizon_yr'
			h = 1;
		case 'v_max_kmh'
			h = 1;
		otherwise
			h = pct_h*params.(fieldname);
	end
end

