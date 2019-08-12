%% Script to test the robot kinematics
clc, clear, close all
addpath('kinematics')
addpath('path-planning')
addpath('utils')

n = 15; % number of cutouts

cutouts.w = 1.19 * ones(1,n) * 1e-3; % [m]
cutouts.u = [1.56 * ones(1,10) * 1e-3, 0.35 * ones(1,5) * 1e-3]; % [m]
cutouts.h = [0.073 * ones(1,10) * 1e-3, 0.40 * ones(1,5) * 1e-3]; % [m]
cutouts.alpha = [zeros(1,10), 7, ones(1,4)];

configuration = [sum(cutouts.h), 0, 0];

robot = Wrist(1.2e-3, 1.4e-3, n, cutouts);
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