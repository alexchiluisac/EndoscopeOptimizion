%% Main function for Notched Tube Designer App
% Created by: Floris van Rossum, 5/28/2019
% COgnitive MEdical Technology and Robotics Laboratory (COMET Robotics Lab)
% Worcester Polytechnic Institute

% Close all previous windows and add folders, sub-folders to path
clc; close all; clear all;
addpath(".");
addpath('cost-functions');
addpath('utils');
addpath('utils/stlTools/');
addpath('utils/visibility/');
addpath('kinematics');
addpath('path-planning');
addpath('utils/ray-casting/');
addpath('gui');

app = NotchedDesigner();
loadRobot(app);

function loadRobot(app)
    cutouts.w = [1 1 1 1];
    cutouts.u = [1 1 1 1];
    cutouts.h = [1 1 1 1];
    cutouts.alpha = [0 0 pi 0];

    configuration = [0.6, 0, 2];

    robot = Wrist(1.6, 1.85, 4, cutouts);
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
