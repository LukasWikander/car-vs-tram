function create_latex_schedule(output,selectedConfiguration)
%CREATE_LATEX_SCHEDULE Summary of this function goes here
%   Detailed explanation goes here
tram_freq = output.results.fleet.tram_freq(selectedConfiguration,:);
hrs = output.params.pass_flow.x;

mins_A2B = NaN(max(tram_freq), length(hrs));
mins_B2A = NaN(max(tram_freq), length(hrs));

rest = [];
for ii = 1:length(tram_freq)
	temp = 0:(60/tram_freq(ii)):59;
	for jj = 1:length(temp)
		mins_A2B(jj,ii) = floor(temp(jj));
	end
	
	
	temp = [rest output.results.tram_full.t_round_trip/2/60 + (0:(60/tram_freq(ii)):59)];
	rest = temp(temp >= 60) - 60;
	temp(temp >= 60) = [];
	for jj = 1:length(temp)
		mins_B2A(jj,ii) = floor(temp(jj));
	end
end

write_preamble('schedule.tex');
write_table('schedule.tex',mins_A2B,hrs);
ff = fopen('schedule.tex','a');
fprintf(ff,'\\columnbreak\n\n');
fprintf(ff,'\\textsf{\\textbf{\\color{Blue}B TO A}} \\vspace{2pt}\n\n');
write_table('schedule.tex',mins_B2A,hrs);
fprintf(ff,'\\end{multicols}\n');
fprintf(ff,'\\end{document}\n');

end

