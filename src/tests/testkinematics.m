%% Script to test the robot kinematics
clc, clear, close all
addpath('kinematics')
addpath('path-planning')
addpath('utils')

cutouts.w = [1 1 1 1] * 1e-3; % [m]
cutouts.u = [1 1 1 1] * 1e-3; % [m]
cutouts.h = [1 1 1 1] * 1e-3; % [m]
cutouts.alpha = [0 0 0 0];

configuration = [1e-3, 0, 0];

robot = Wrist(1.6e-3, 1.85e-3, 4, cutouts);
robot.fwkine(configuration, eye(4));
robotModel = robot.makePhysicalModel();
P = robot.pose(:,end);
T = robot.transformations;

X = P(1,:);
Y = P(2,:);
Z = P(3,:);

figure
scatter3(X, Y, Z, 100, 'r', 'filled');
hold on, axis equal

plot3(X, Y, Z, 'k', 'LineWidth', 2.5);
plot3(robotModel.backbone(1,:), ...
      robotModel.backbone(2,:), ...
      robotModel.backbone(3,:), ...
      'LineWidth', 2.5);
  
xlabel('X[mm]')
ylabel('Y[mm]')
zlabel('Z[mm]')

for ii = 1 : size(T,3)
    triad('Matrix', T(:,:,ii), 'linewidth', 2.5, 'scale', 1e-4);    
end

figure
scatter3(X, Y, Z, 100, 'r', 'filled');
hold on, axis equal

plot3(X, Y, Z, 'k', 'LineWidth', 2.5);
xlabel('X[mm]')
ylabel('Y[mm]')
zlabel('Z[mm]')

triad('Matrix', eye(4), 'linewidth', 2.5, 'scale', 1e-4);
triad('Matrix', T(:,:,end), 'linewidth', 2.5,  'scale', 1e-4);

X = robotModel.surface.X;
Y = robotModel.surface.Y;
Z = robotModel.surface.Z;
surf(X, Y, Z, 'FaceColor','blue');

X = robotModel.surface.X;
Y = robotModel.surface.Y;
Z = robotModel.surface.Z;
surf(X, Y, Z, 'FaceColor','blue');