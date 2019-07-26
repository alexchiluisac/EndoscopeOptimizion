%% Script to test the robot kinematics
clc, clear, close all
addpath('kinematics')
addpath('path-planning')
addpath('utils')

cutouts.w = [1 1 1 1 1 1 1 1 1 1];
cutouts.u = [1 1 1 1 1 1 1 1 1 1];
cutouts.h = [1 1 1 1 1 1 1 1 1 1];
%cutouts.alpha = [0 0 pi/3 0 0 0 pi/3 0 0 0];
cutouts.alpha = [0 0 0 0 0 0 0 0 0 0];

configuration = [5, 0, 2];

robot = Wrist(1.6, 1.85, 10, cutouts);
robot.fwkine(configuration, eye(4));

X = robot.pose(1,:) * 1e-3;
Y = robot.pose(2,:) * 1e-3;
Z = robot.pose(3,:) * 1e-3;

figure
scatter3(X, Y, Z, 100, 'r', 'filled');
hold on, axis equal

plot3(X, Y, Z, 'k', 'LineWidth', 2.5);
xlabel('X[mm]')
ylabel('Y[mm]')
zlabel('Z[mm]')

T = robot.transformations;
T(1:3,end,:) = T(1:3,end,:) .* 1e-3;

for ii = 1 : size(T,3)
    triad('Matrix', T(:,:,ii), 'linewidth', 2.5, 'scale', 1e-3/2);    
end
%triad('Matrix', eye(4), 'linewidth', 2.5);
%triad('Matrix', T(:,:,end), 'linewidth', 2.5);

figure
scatter3(X, Y, Z, 100, 'r', 'filled');
hold on, axis equal

plot3(X, Y, Z, 'k', 'LineWidth', 2.5);
xlabel('X[mm]')
ylabel('Y[mm]')
zlabel('Z[mm]')

triad('Matrix', eye(4), 'linewidth', 2.5, 'scale', 1e-3/2);
triad('Matrix', T(:,:,end), 'linewidth', 2.5, 'scale', 1e-3/2);

robotModel = robot.makePhysicalModel();

X = robotModel.surface.X;
Y = robotModel.surface.Y;
Z = robotModel.surface.Z;
surf(X, Y, Z, 'FaceColor','green');

