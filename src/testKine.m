%% Robot endoscope features.
clc, clear, close all
addpath('kinematics')
addpath('path-planning')
addpath('utils')

cutouts(1).w = 1;
cutouts(1).u = 1;
cutouts(1).h = 1;
cutouts(1).alpha = 0;

configuration = [0.2, 0, 2];

robot = Wrist(1.6, 1.85, 4, cutouts);
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

%T = eye(4);
%R = [0 0 1; 0 1 0; -1 0 0];
%T(1:3,1:3) = R;
%bb = applytransform(bb, T);

X = bb(1,:);
Y = bb(2,:);
Z = bb(3,:);

scatter3(X, Y, Z, 100, 'b', 'filled');

radius = robot.OD/2*ones(1,size(bb,2));
[X,Y,Z] = gencyl(bb,radius);
cyl = surf(X,Y,Z,'FaceColor','blue');