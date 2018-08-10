function write_table(filename,tabledata,hrs)
n_mins = size(tabledata,1);
ff = fopen(filename,'a');

fprintf(ff,'\\begin{tabular}{a');
for ii = 1:n_mins
	fprintf(ff,'b');
end
fprintf(ff,'}\n\n');
fprintf(ff,'\\rowcolor{Blue}\n');
fprintf(ff,'\\multicolumn{%2.0f}{l}{\\color{white}MONDAY - FRIDAY} \\\\',n_mins+1);
fprintf(ff,'\\rowcolor{white}\n');
fprintf(ff,'\\ & & & & & & \\\\[-2ex]\n');
for h_no = 1:size(tabledata,2)
	fprintf(ff,'\\color{white}%02.0f ',hrs(h_no));
	for ii = 1:n_mins
		if isnan(tabledata(ii,h_no))
			fprintf(ff,'& ');
		else
			fprintf(ff,'& %02.0f ',tabledata(ii,h_no));
		end
	end
	
	fprintf(ff,'\\\\\n');

end

fprintf(ff,'\\end{tabular}\n');
fprintf(ff,'\\vfill\\null\n');


fclose(ff);
end

