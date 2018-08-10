function write_preamble(filename)
ff = fopen(filename,'w');
fprintf(ff,'\\documentclass[a4paper,12pt]{article}\n');
fprintf(ff,'\\usepackage{xcolor,colortbl}\n');
fprintf(ff,'\\usepackage{multicol}\n');
fprintf(ff,'\\usepackage{geometry}\n');
fprintf(ff,'\\geometry{legalpaper, margin=1in}');
fprintf(ff,'\n');
fprintf(ff,'\\definecolor{Blue}{rgb}{0.3529,0.7216,0.9294}\n');
fprintf(ff,'\\newcolumntype{a}{>{\\columncolor{Blue}}c}\n');
fprintf(ff,'\\newcolumntype{b}{>{\\columncolor{white}}c}\n');
fprintf(ff,'\\renewcommand{\\familydefault}{\\ttdefault}\n');
fprintf(ff,'\\begin{document}\n');
fprintf(ff,'\\pagenumbering{gobble}\n');
fprintf(ff,'\n');
fprintf(ff,'\\begin{multicols}{2}\n');
fprintf(ff,'\\textsf{\\textbf{\\color{Blue}A TO B}} \\vspace{2pt}\n\n');


fclose(ff);
end

