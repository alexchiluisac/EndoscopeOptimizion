%% Robot endoscope features.
clc, clear, close all
addpath('kinematics')
addpath('path-planning')
addpath('utils')

cutouts(1).w = 1;
cutouts(1).u = 1;
cutouts(1).h = 1;
cutouts(1).alpha = 0;

robot = Wrist(1.6, 1.85, 3, cutouts);
%teach(robot, [0.2, 0, 0]);

maxDisplacement = 1; % [mm]
maxRotation     = 2*pi; % [rad]
maxAdvancement  = 10; % [mm]
[qListNormalized,qList,pList,aList] = rrt(robot, ...
    [maxDisplacement maxRotation maxAdvancement]);

figure
scatter3(qList(1,:), qList(2,:), qList(3,:));
axis equal, grid on
xlabel('Pull-wire displacement [mm]');
ylabel('Axial rotation [rad]');
zlabel('Axial translation [mm]');
title('Configurations generated by RRT');

figure
scatter3(qListNormalized(1,:), qListNormalized(2,:), qListNormalized(3,:));
axis equal, grid on
xlabel('Pull-wire displacement [mm]');
ylabel('Axial rotation [rad]');
zlabel('Axial translation [mm]');
title('Configurations generated by RRT (normalized)');

figure
scatter3(pList(1,:), pList(2,:), pList(3,:));
axis equal, grid on
xlabel('X [mm]'), ylabel('Y [mm]'), zlabel('Z [mm]');
title('Reachable points in the task space');


% Load ear model
path = fullfile('..', 'anatomical-models', 'synthetic-model.stl');
[vertices, faces, ~, ~] = stlRead(path);
earModel.vertices = vertices;
earModel.faces = faces;

% Visualize larynx model
plotHandle = stlPlot(vertices, faces, 'Synthetic Model');
hold on;
view([17.8 30.2]);

t = [30 10 10];
R = [0 0 -1; 0 1 0; 1 0 0];
T = [R t'; 0 0 0 1];


[P, ~] = robot.fwkine(qList(:,500));
P = applytransform(P, T);

X = P(1,:);
Y = P(2,:);
Z = P(3,:);
  
scatter3(X, Y, Z, 10, 'r', 'filled');
hold on, axis equal
  
plot3(X, Y, Z, 'k', 'LineWidth', 2.5);
xlabel('X[mm]')
ylabel('Y[mm]')
zlabel('Z[mm]')
  
%triad('Matrix', eye(4), 'linewidth', 2.5);
triad('Matrix', T(:,:,end), 'linewidth', 2.5);

%Surrounds backbone with cylinders  
scatter3(X, Y, Z, 100, 'b', 'filled');

radius = robot.OD/2*ones(1,size(P,2));
[X,Y,Z] = gencyl(P,radius);
cyl = surf(X,Y,Z,'FaceColor','blue');

%intriangulation: check collision points between anatomical model and
%endoscope
testp = [X(:) Y(:) Z(:)];
testintrian = intriangulation(vertices,faces,testp);
h = trisurf(faces,vertices(:,1),vertices(:,2),vertices(:,3));
set(h,'FaceColor','black','FaceAlpha',1/3,'EdgeColor','none');
hold on
plot3(testp(:,1),testp(:,2),testp(:,3),'b.');
plot3(testp(testintrian==1,1),testp(testintrian==1,2),testp(testintrian==1,3),'ro');