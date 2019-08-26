function estimateReachableWorkspace(u,h,n,minAdvancement)
%% This function estimates the reachable workspace for a given endoscope configuration
% Inputs:
%         u: length of uncut sections [mm]
%         h: length of cut sections   [mm]
%         n: number of cutouts
%         minAdvancement: distance between base of the robot and entry
%         point into the ear, as defined in configurations.txt

if nargin < 1
    u = 0.92; % [mm]
    h = 0.17; % [mm]
    n = 3;    
    minAdvancement = 0;
end

% How many configuration points should we sample for testing?
nPoints = 10;

% Which anatomical model should we use?
modelID = 'atlas';

fprintf('*** RRT and estimation of reachable workspace ***\n')

% add dependencies
addpath('kinematics')
addpath('utils')
addpath('utils/stlTools')
addpath('path-planning')
addpath('../anatomical-models')

%% Part 1. Step-by-step testing of RRT
fprintf('Running RRT...\n')

% Define the endoscope model
cutouts.w = 1.36 * ones(1,n) * 1e-3; % [m]
cutouts.u = [u * ones(1,n-1) * 1e-3, 4.5 * 1e-3]; % [m]
cutouts.h = h * ones(1,n) * 1e-3; % [m]
cutouts.alpha = zeros(1,n);

% Load ear model
% Read the configuration file to extract information about the
% meshes
fid = fopen(fullfile('..', 'anatomical-models', 'configurations.txt'));
text = textscan(fid, '%s %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f');
fclose(fid);

configurations = cell2mat(text(2:end));
line_no = find(strcmp(text{1}, modelID));

path = fullfile('..', 'anatomical-models', modelID);
%path       = fullfile('meshes', text{1}(line_no));
%path       = path{:}; % converts cell to char
image_size   = configurations(line_no, 1:3);
voxel_size   = configurations(line_no, 4:6);
entry_point  = configurations(line_no, 7:9);
tip_base     = configurations(line_no, 10:12);
%target_point = configurations(line_no, 13:15);

% Calculate the transformation to the space of the CT scan
newZ = tip_base .* 1e-3 - entry_point .* 1e-3;
newZ = newZ ./ norm(newZ);
v = cross([0 0 1], newZ);
R = eye(3) + skew(v) + skew(v)^2 * (1-dot([0 0 1], newZ))/norm(v)^2;
t = entry_point .* 1e-3;
T = [R t'; 0 0 0 1];

% now slide the endoscope back by its length, so that all the different
% designs start exploring from the same point
Tz = eye(4);
Tz(3,4) = -(sum(cutouts.u) + sum(cutouts.h));
T = T * Tz;

% Read the meshes from file
pathStl = fullfile(path, 'me-solid.stl');
[vertices, faces, ~, ~] = stlRead(pathStl);
earModel.vertices = vertices;
earModel.faces = faces;
earModel.baseTransform = T;

pathStl = fullfile(path, 'ossicle.stl');
[vertices, faces, ~, ~] = stlRead(pathStl);
osModel.vertices = vertices;
osModel.faces = faces;

% Define the robot's range of motion
maxDisplacement = sum(cutouts.h);  % [m]
maxRotation     = 3*pi;  % [rad]
minAdvancement  = minAdvancement * 1e-3;  % [m]
maxAdvancement  = 20e-3-minAdvancement; % [m]

robot = Wrist(1.4e-3, 1.6e-3, n, cutouts);

[qListNormalized,qList,pList,aList] = rrt(robot, ...
    [0 maxDisplacement 0 maxRotation minAdvancement maxAdvancement], ...
    earModel, ...
    osModel, ...
    nPoints);

fprintf(['RRT execution complete. Total sampled points: ' num2str(size(qList,2)) ' \n\n']);

fprintf('\n Generating reachable workspace...\n')
shrinkFactor = 1;
[k,v] = boundary(pList(1,:)', pList(2,:)', pList(3,:)', shrinkFactor);

figure
scatter3(qList(1,:), qList(2,:), qList(3,:));
grid on
xlabel('Pull-wire displacement [mm]');
ylabel('Axial rotation [rad]');
zlabel('Axial translation [mm]');
title('Configurations generated by RRT');

figure
scatter3(qListNormalized(1,:), qListNormalized(2,:), qListNormalized(3,:));
grid on
xlabel('Pull-wire displacement [m]');
ylabel('Axial rotation [rad]');
zlabel('Axial translation [m]');
title('Configurations generated by RRT (normalized)');

% Visualize ear model
figure, hold on
stlPlot(earModel.vertices, earModel.faces, 'Ear Model');
stlPlot(osModel.vertices, osModel.faces, 'Ear Model');
%view([17.8 30.2]);

scatter3(pList(1,:), pList(2,:), pList(3,:),'red','filled');
axis equal, grid on
xlabel('X [m]'), ylabel('Y [m]'), zlabel('Z [m]');
title('Reachable points in the task space');

figure, hold on
stlPlot(earModel.vertices, earModel.faces, 'Ear Model');
stlPlot(osModel.vertices, osModel.faces, 'Ear Model');
trisurf(k, pList(1,:)', pList(2,:)', pList(3,:)','FaceColor','red','FaceAlpha',0.1)
title('Reachable workspace');

fprintf('Testing complete.\n')

save([num2str(n) '-simulation.mat']);
end