function printOutParameters(app)
%OUTPUTPARAMETERS Output the notch parameters
%   Print out the notch parameters to the console and to the text file

nCutouts = app.wrist.nCutouts;
cutouts = app.wrist.cutouts;

differentCutouts = zeros(nCutouts,4);
for i= 1:nCutouts
    subarr = [cutouts.u(1, i), cutouts.alpha(1, i), cutouts.w(1, i), cutouts.h(1, i)];
    differentCutouts(i, :) = subarr;
end

disp("Creating file");
fileID = fopen('output.txt', 'wt' );
if fileID == -1
  error('Author:Function:OpenFile', 'Cannot open file: %s', fileID);
end

% Introduction
fprintf(fileID, ' --- Notched Endoscope Research Design Software --- \n');
fprintf(fileID, 'COgnitive MEdical Technology (COMET) Laboratory \n');
fprintf(fileID, 'Professor Loris Fichera \n');
fprintf(fileID, 'Software by: Floris van Rossum \n');
fprintf(fileID, '\n\n');
fprintf(fileID, ' --- INSTRUCTIONS --- \n');
fprintf(fileID, ' \n');
fprintf(fileID, ' --- YOUR NOTCHED TUBE --- \n');
fprintf(fileID, 'Created on: %s at, %s \n', datetime('now'));

fprintf(fileID, 'Tube Parameters: \n');

% Print different cutout statistics
for i= 1:nCutouts
    fprintf(fileID, 'Notch %d \n', i);
    fprintf(fileID, '\t Spacing: %d mm \n', differentCutouts(i, 1));
    fprintf(fileID, '\t Alpha:   %d rad\n', differentCutouts(i, 1));
    fprintf(fileID, '\t Width:   %d mm\n', differentCutouts(i, 1));
    fprintf(fileID, '\t Height:  %d mm\n', differentCutouts(i, 1));
    fprintf(fileID, '\n');
end


% Close the file
fclose(fileID);
end

