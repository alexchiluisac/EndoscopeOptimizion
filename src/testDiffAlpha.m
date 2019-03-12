%% Robot endoscope features.
clc, clear, close all
addpath('kinematics')
addpath('path-planning')
addpath('utils')
addpath('../anatomical-models')

cutouts(1).w = 1;
cutouts(1).u = 1;
cutouts(1).h = 1;
cutouts(1).alpha = 0;



maxDisplacement = 1; % [mm]
maxRotation     = 2*pi; % [rad]
maxAdvancement  = 10; % [mm]

% Load ear model
path = fullfile('..', 'anatomical-models', 'synthetic-model.stl');
[vertices, faces, ~, ~] = stlRead(path);
earModel.vertices = vertices;
earModel.faces = faces;

% Calculate base transform
t = [30 10 10];
R = [0 0 -1; 0 1 0; 1 0 0];
T = [R t'; 0 0 0 1];
earModel.baseTransform = T;

% Estimate the reachable volume for different alpha values.

reachableVolume = zeros(5,1);
i = 1;

for alpha = 0:pi/4:pi
 
% Create the robot
robot = Wrist(1.6, 1.85, 4, cutouts);

[qListNormalized,qList,pList,aList] = rrt(robot, ...
    [maxDisplacement maxRotation maxAdvancement], ...
    earModel);

[k,v] = boundary(pList(1,:)', pList(2,:)', pList(3,:)', 0.5);
angles(i,1) = alpha;
reachableVolume(i,1) = v;
i = i + 1;

end

% Plot objective function
angles = angles*180/pi;
figure
plot(angles,reachableVolume)
axis equal, grid on
xlabel('Angle [Degree]'), ylabel('Reachable Volume [mm3]');


% figure
% scatter3(qList(1,:), qList(2,:), qList(3,:));
% axis equal, grid on
% xlabel('Pull-wire displacement [mm]');
% ylabel('Axial rotation [rad]');
% zlabel('Axial translation [mm]');
% title('Configurations generated by RRT');
% 
% figure
% scatter3(qListNormalized(1,:), qListNormalized(2,:), qListNormalized(3,:));
% axis equal, grid on
% xlabel('Pull-wire displacement [mm]');
% ylabel('Axial rotation [rad]');
% zlabel('Axial translation [mm]');
% title('Configurations generated by RRT (normalized)');

% Visualize ear model
plotHandle = stlPlot(vertices, faces, 'Synthetic Model');
hold on;
view([17.8 30.2]);

%figure
scatter3(pList(1,:), pList(2,:), pList(3,:),'red','filled');
trisurf(k, pList(1,:)', pList(2,:)', pList(3,:)','FaceColor','red','FaceAlpha',0.1)
axis equal, grid on
xlabel('X [mm]'), ylabel('Y [mm]'), zlabel('Z [mm]');
title('Reachable points in the task space');


return


[P, ~] = robot.fwkine(qList(:,500),T);
%P = applytransform(P, T);

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
%scatter3(X, Y, Z, 10, 'b', 'filled');

%radius = robot.OD/2*ones(1,size(P,2));
%[X,Y,Z] = gencyl(P,radius);
robotPM = robot.makePhysicalModel(qList(:,500), T);
surf(robotPM.surface.X, robotPM.surface.Y, robotPM.surface.Z, 'FaceColor','blue');

%intriangulation: check collision points between anatomical model and
%endoscope
testp = [X(:) Y(:) Z(:)];
intrian = intriangulation(vertices,faces,testp);
h = trisurf(faces,vertices(:,1),vertices(:,2),vertices(:,3));
set(h,'FaceColor','black','FaceAlpha',1/3,'EdgeColor','none');
hold on
plot3(testp(:,1),testp(:,2),testp(:,3),'b.','MarkerSize',1);
plot3(testp(intrian==1,1),testp(intrian==1,2),testp(intrian==1,3),'ro');