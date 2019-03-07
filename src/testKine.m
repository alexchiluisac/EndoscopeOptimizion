%% Robot endoscope features.
clc, clear, close all
addpath('kinematics')
addpath('path-planning')
addpath('utils')

cutouts(1).w = 1;
cutouts(1).u = 1;
cutouts(1).h = 1;
cutouts(1).alpha = 0;

configuration = [0.2, pi/4, 2];

robot = Wrist(1.6, 1.85, 3, cutouts);
[P, T] = robot.fwkine(configuration);

X = P(1,:);
Y = P(2,:);
Z = P(3,:);

figure
scatter3(X, Y, Z, 100, 'r', 'filled');
hold on, axis equal

plot3(X, Y, Z, 'k', 'LineWidth', 2.5);
xlabel('X[mm]')
ylabel('Y[mm]')
zlabel('Z[mm]')

triad('Matrix', eye(4), 'linewidth', 2.5);
triad('Matrix', T(:,:,end), 'linewidth', 2.5);


bb = robot.makePhysicalModel(configuration);

X = bb(1,:);
Y = bb(2,:);
Z = bb(3,:);

scatter3(X, Y, Z, 100, 'b', 'filled');

