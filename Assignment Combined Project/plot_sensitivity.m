function plot_sensitivity(sensitivity, general_params)
	for param_no = 1:3
		switch param_no
			case 1
				param = 'general_params';
				name = 'general parameters';
			case 2
				param = 'tram_params';
				name = 'tram parameters';
			case 3
				param = 'car_params';
				name = 'car parameters';
		end
		
		%% Extract relevant data
		barnames = fieldnames(sensitivity.(param));
		for ii = 1:length(barnames)
			if ~isnan(sensitivity.(param).(barnames{ii}))
				bardata = zeros(length(barnames),1);
				break
			end
		end
		
		for ii = 1:length(bardata)
			try
				bardata(ii) = abs(sensitivity.(param).(barnames{ii})(1));
				if abs(sensitivity.(param).(barnames{ii})(2)) > 0
					warning(['Fleet composition is sensitive to ' barnames{ii} ' at specified year!']);
				end
			catch
				bardata(ii) = NaN;
			end
		end
		% Remove NaN values from plot
		for ii = length(bardata):-1:1
			if any(isnan(bardata(ii)))
				bardata(ii) = [];
				barnames(ii) = [];
			end
		end
		
		
		%% Plot
		figure('Name',['Sensitivity of ' num2str(general_params.ROI_horizon_yr) ' year cost to ' name])
		barh(bardata)
		set(gca,'yticklabel',barnames)
		set(gca,'ticklabelinterpreter','none')
		set(gca,'ytick',1:length(barnames))
		set(gca,'xscale','log')
		xlabel('Magnitude of sensitivity [SEK/Unit]')
		title(['Sensitivity of cost to ' name])
		grid on
	end
	
end

