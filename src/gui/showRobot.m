function showRobot(app, var)
%SHOWROBOT Load and display the robot in the app GUI
%   Use the various robot modeling functions to load and display the robot
%   within the app PlotAxes object. Called whenever the display updates.
    cla(app.PlotAxes);
    cutouts.w = [1 1 1 1];
    cutouts.u = [1 1 1 1];
    cutouts.h = [1 1 1 1];
    cutouts.alpha = [0 0 pi 0];

    configuration = [0.6, 0, 2];

    robot = Wrist(var, 1.85, 4, cutouts);
    [P, T] = robot.fwkine(configuration, eye(4));

    X = P(1,:);
    Y = P(2,:);
    Z = P(3,:);
    
    % Draw red circles

    result = scatter3(app.PlotAxes, X, Y, Z, 100, 'r', 'filled');
    
    % Draw black line
    plot3(app.PlotAxes, X, Y, Z, 'k', 'LineWidth', 2.5);
    
    robotModel = robot.makePhysicalModel(configuration, eye(4));

    X = robotModel.surface.X;
    Y = robotModel.surface.Y;
    Z = robotModel.surface.Z;
    surf(app.PlotAxes, X, Y, Z, 'FaceColor','green');
    app.PlotAxes.View = [-135 35];
end

