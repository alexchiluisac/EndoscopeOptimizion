%% Script to test RRT and collision detection
clc, clear, close all

% How many configuration points should we sample for testing?
nPoints = 2;
modelID = 'atlas';

fprintf('*** Ray Casting test ***\n')
fprintf('This script uses a ray-casting algorithm to estimate the visual range of our robot.\n')
fprintf('Press any key to continue.\n')
pause

% add dependencies
addpath('kinematics')
addpath('path-planning')
addpath('utils')
addpath('utils/ray-casting')
addpath('utils/stlTools')
addpath('utils/visibility')
addpath('../anatomical-models')


% define the robot's range of motion
maxDisplacement = 1e-3; % [m]
maxRotation     = 4*pi;    % [rad]
maxAdvancement  = 10e-3;   % [m]

% Load ear model
% Read the configuration file to extract information about the
% meshes
fid = fopen(fullfile('..', 'anatomical-models', 'configurations.txt'));
text = textscan(fid, '%s %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f');
fclose(fid);

configurations = cell2mat(text(2:end));
line_no = find(strcmp(text{1}, modelID));

path       = fullfile('meshes', text{1}(line_no));
path       = path{:}; % converts cell to char
image_size   = configurations(line_no, 1:3);
voxel_size   = configurations(line_no, 4:6);
entry_point  = configurations(line_no, 7:9);
tip_base     = configurations(line_no, 10:12);
%target_point = configurations(line_no, 13:15);

% Read the Raw Meshes from file
pathMe = fullfile('..', 'anatomical-models', modelID, 'me.mesh');
pathOs = fullfile('..', 'anatomical-models', modelID, 'ossicle.mesh');
rawMeMesh = meshread(pathMe);
rawOsMesh = meshread(pathOs);

% Convert the raw meshes into objects that can be passed
% to the `patch' function
meMesh.faces = rawMeMesh.triangles' + 1;
meMesh.vertices = bsxfun(@times, rawMeMesh.vertices', voxel_size);
meMesh.vertices = meMesh.vertices .* 1e-3;

osMesh.faces = rawOsMesh.triangles' + 1;
osMesh.vertices = bsxfun(@times, rawOsMesh.vertices', voxel_size);
osMesh.vertices = osMesh.vertices .* 1e-3;

% Set the transformation between curve and model space
newZ = tip_base .* 1e-3 - entry_point .* 1e-3;
newZ = newZ ./ norm(newZ);
v = cross([0 0 1], newZ);
R = eye(3) + skew(v) + skew(v)^2 * (1-dot([0 0 1], newZ))/norm(v)^2;
t = entry_point .* 1e-3;
T = [R t'; 0 0 0 1];

meMesh.baseTransform = T;

% Create a robot
alpha = 0;
cutouts = [];
cutouts.w = [1.1 1.1 1.1 1.1] * 1e-3;
cutouts.u = [1 1 1 1] * 1e-3;
cutouts.h = [1 1 1 1] * 1e-3;
cutouts.alpha = [0 0 0 0 0 0];
robot = Wrist(1e-3, 1.2e-3, 4, cutouts);

fprintf('Running RRT...\n')

[~,qList,pList,aList] = rrt(robot, ...
    [maxDisplacement maxRotation maxAdvancement], ...
    meMesh, ...
    nPoints);

fprintf(['RRT execution complete. Total sampled points: ' num2str(size(pList,2)) ' \n\n']);

figure

% Visualize the visual range for each pose of the robot
ii = 1;
seenMap = visibilitymap(pList(:,ii), aList(:,ii), meMesh, osMesh);
h1 = stlPlot(earModel.vertices, earModel.faces, 'Ray Casting test.', seenMap);
hold on


robotPhysicalModel = robot.makePhysicalModel();
h2 = surf(robotPhysicalModel.surface.X, ...
    robotPhysicalModel.surface.Y, ...
    robotPhysicalModel.surface.Z, ...
    'FaceColor','blue');

axis equal

while true
    seenMap = visibilitymapsynthetic(pList(:,ii), aList(:,ii), meMesh, osMesh);
    robot.fwkine(qList(:,ii), T);
    robotPhysicalModel = robot.makePhysicalModel();
    
    colorMap = zeros(length(h1.FaceVertexCData), 1);
    colorMap(logical(seenMap)) = 5;
    h1.FaceVertexCData = colorMap;
    
    h2.XData = robotPhysicalModel.surface.X;
    h2.YData = robotPhysicalModel.surface.Y;
    h2.ZData = robotPhysicalModel.surface.Z;
    title(['Pose ' num2str(ii) ' of ' num2str(size(pList, 2))]);
    
    fprintf(['Seen area: ' num2str(seenArea(earModel, seenMap)) ' m2.\n']);
    fprintf('Press "n" to move forward or "p" to move back.\n')
    fprintf('Press any other key to exit.\n\n')
    
    while ~waitforbuttonpress, end
    k = get(gcf, 'CurrentCharacter');
    
    switch k
        case 'p'
            ii = ii - 1;
            if ii < 1, ii = 1; end
        case 'n'
            ii = ii + 1;
            if ii > size(pList, 2), ii = size(pList, 2); end
        case '-'
            ii = ii + 10;
            if ii < 1, ii = 1; end
        case '+'
            ii = ii + 10;
            if ii > size(pList, 2), ii = size(pList, 2); end
        otherwise
            break
    end
end

fprintf('Testing complete.\n')