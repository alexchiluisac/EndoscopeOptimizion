function showSTL(app, fileName, filePath)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
% Load cavity model (finer mesh this time)
    path = fullfile( filePath, fileName);
    disp(path)
    [vertices, faces, ~, ~] = stlRead(path);
    earModel.vertices = vertices;
    earModel.faces = faces;
    stlGUIPlot(app.PlotAxes, earModel.vertices, earModel.faces, 'Collision detection test.');
end

