%% Script to test the robot kinematics
clc, clear, close all
addpath('kinematics')
addpath('path-planning')
addpath('utils')

cutouts.w = [1];
cutouts.u = [1];
cutouts.h = [1];
cutouts.alpha = [0];

configuration = [1, 0, 0];

robot = Wrist(1.6, 1.85, 1, cutouts);
robot.fwkine(configuration, eye(4));
J = robot.jacob0(configuration)

X = robot.pose(1,:);
Y = robot.pose(2,:);
Z = robot.pose(3,:);

figure
scatter3(X, Y, Z, 100, 'r', 'filled');
hold on, axis equal

plot3(X, Y, Z, 'k', 'LineWidth', 2.5);
xlabel('X[mm]')
ylabel('Y[mm]')
zlabel('Z[mm]')

for ii = 1 : size(robot.transformations,3)
    triad('Matrix', robot.transformations(:,:,ii), 'linewidth', 2.5);    
end

figure
scatter3(X, Y, Z, 100, 'r', 'filled');
hold on, axis equal

plot3(X, Y, Z, 'k', 'LineWidth', 2.5);
xlabel('X[mm]')
ylabel('Y[mm]')
zlabel('Z[mm]')

triad('Matrix', eye(4), 'linewidth', 2.5);
triad('Matrix', robot.transformations(:,:,end), 'linewidth', 2.5);

robotModel = robot.makePhysicalModel();

X = robotModel.surface.X;
Y = robotModel.surface.Y;
Z = robotModel.surface.Z;
surf(X, Y, Z, 'FaceColor','green');

