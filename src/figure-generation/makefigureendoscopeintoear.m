addpath('kinematics')
addpath('path-planning')
addpath('utils')

load 5-simulation.mat

n = 5;
w = 1.36;
u = 0.84;
h = 0.34;

cutouts.w = 1.36 * ones(1,n) * 1e-3; % [m]
cutouts.u = [u * ones(1,n-1) * 1e-3, 4.5 * 1e-3]; % [m]
cutouts.h = h * ones(1,n) * 1e-3; % [m]
cutouts.alpha = zeros(1,n);
cutouts.alpha = zeros(1,n);
robot = Wrist(1.4e-3, 1.6e-3, n, cutouts);


% Load ear model
% Read the configuration file to extract information about the
% meshes
fid = fopen(fullfile('..', 'anatomical-models', 'configurations.txt'));
text = textscan(fid, '%s %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f');
fclose(fid);

configurations = cell2mat(text(2:end));
line_no = find(strcmp(text{1}, modelID));

path = fullfile('..', 'anatomical-models', modelID);
image_size   = configurations(line_no, 1:3);
voxel_size   = configurations(line_no, 4:6);
entry_point  = configurations(line_no, 7:9);
tip_base     = configurations(line_no, 10:12);

newZ = tip_base .* 1e-3 - entry_point .* 1e-3;
newZ = newZ ./ norm(newZ);
v = cross([0 0 1], newZ);
R = eye(3) + skew(v) + skew(v)^2 * (1-dot([0 0 1], newZ))/norm(v)^2;
t = entry_point .* 1e-3;
T = [R t'; 0 0 0 1];

pathStl = fullfile(path, 'me-solid.stl');
[vertices, faces, ~, ~] = stlRead(pathStl);
earModel.vertices = vertices;
earModel.faces = faces;
earModel.baseTransform = T;



configuration = qList(:,25);
robot.fwkine(configuration, T);
robotModel = robot.makePhysicalModel();

figure('units','normalized','outerposition',[0 0 1 1])
hold on
pathStl = fullfile(path, 'me.stl');
[vertices, faces, ~, ~] = stlRead(pathStl);
earModel.vertices = vertices;
earModel.faces = faces;
stlPlot(earModel.vertices, earModel.faces, 'Ear Model');
stlPlot(osModel.vertices, osModel.faces, 'Ear Model', 10);
%view([17.8 30.2]);

X = robotModel.surface.X;
Y = robotModel.surface.Y;
Z = robotModel.surface.Z;
surf(X, Y, Z, 'FaceColor','blue');
axis equal
xlabel('X[m]')
ylabel('Y[m]')
zlabel('Z[m]')
view(-76.12, 56.23);
title(['Simulation of endoscope with ' num2str(n) ' notches']);
set(gca,'FontSize',18);