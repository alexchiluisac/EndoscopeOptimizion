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
fprintf(fileID, 'Worcester Polytechnic Institute (WPI) \n');
fprintf(fileID, 'Professor Loris Fichera \n');
fprintf(fileID, 'Software by: Floris van Rossum \n');
fprintf(fileID, 'Repository: https://github.com/comet-lab/Shoemaker-Levy\n');
fprintf(fileID, '\n\n');

% print instructions
fprintf(fileID, ' --- INSTRUCTIONS --- \n');
fprintf(fileID, '\tThe notched tube is described below. Notches are numbered starting at 1 \n');
fprintf(fileID, 'and up to the specified number of notches. The lowest numbered notches \n');
fprintf(fileID, 'are at the base of the Notched Endoscope, and the higher numbered notches\n');
fprintf(fileID, 'are closer to the distal end.\n');
fprintf(fileID, 'Read more instructions at: \n\n\n');

% Print different cutout statistics
fprintf(fileID, ' --- YOUR NOTCHED TUBE --- \n');
fprintf(fileID, 'Created on: %s \n', datetime('now'));
fprintf(fileID, 'Notched Endoscope Parameters: \n');

for i= 1:nCutouts
    fprintf(fileID, 'Notch %d Parameters: \n', i);
    fprintf(fileID, '\t Spacing: %d mm \n', differentCutouts(i, 1));
    fprintf(fileID, '\t Alpha:   %d radians\n', differentCutouts(i, 1));
    fprintf(fileID, '\t Width:   %d mm\n', differentCutouts(i, 1));
    fprintf(fileID, '\t Height:  %d mm\n', differentCutouts(i, 1));
    fprintf(fileID, '\n');
end

% Close the file
fclose(fileID);

% Introduction
fprintf(' --- Notched Endoscope Research Design Software --- \n');
fprintf('COgnitive MEdical Technology (COMET) Laboratory \n');
fprintf('Worcester Polytechnic Institute (WPI) \n');
fprintf('Professor Loris Fichera \n');
fprintf('Software by: Floris van Rossum \n');
fprintf('https://github.com/comet-lab/Shoemaker-Levy\n');
fprintf('\n\n');

% print instructions
fprintf(' --- INSTRUCTIONS --- \n');
fprintf('\tThe notched tube is described below. Notches are numbered starting at 1 \n');
fprintf('and up to the specified number of notches. The lowest numbered notches \n');
fprintf('are at the base of the Notched Endoscope, and the higher numbered notches\n');
fprintf('are closer to the distal end.\n');
fprintf('Read more instructions at: \n\n\n');

% Print different cutout statistics
fprintf(' --- YOUR NOTCHED TUBE --- \n');
fprintf('Created on: %s \n\n', datetime('now'));
fprintf('Notched Endoscope Parameters: \n');

for i= 1:nCutouts
    fprintf('Notch %d Parameters: \n', i);
    fprintf('\t Spacing: %d mm \n', differentCutouts(i, 1));
    fprintf('\t Alpha:   %d radians\n', differentCutouts(i, 1));
    fprintf('\t Width:   %d mm\n', differentCutouts(i, 1));
    fprintf('\t Height:  %d mm\n', differentCutouts(i, 1));
    fprintf('\n');
end

end

