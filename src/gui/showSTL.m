function showSTL(app, fileName, filePath)
%SHOWSTL Load and display the .STL file in the app GUI
%   Use stlGUIPlot function to load and display the .STL file in the app
%   PlotAxes object. Called whenever the display updates.
    path = fullfile( filePath, fileName);
    % fprintf("Loaded Path: %s\n", path);
    
    % Check to ensure that file is loaded
    if strlength(path) > 3
        [vertices, faces, ~, ~] = stlRead(path);
        earModel.vertices = vertices;
        earModel.faces = faces;
        stlGUIPlot(app.PlotAxes, earModel.vertices, earModel.faces);
    else
       % Print error, file path too short or no file loaded
       fprintf("Error: No file loaded or file-path too short!\n") 
    end
    
    % Update the rest of the visuals
    updateGUI(app);
end

