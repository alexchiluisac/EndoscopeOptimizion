%% Script to test RRT and collision detection
clc, clear, close all

% How many configuration points should we sample for testing?
nPoints = 50;

fprintf('*** Ray Casting test ***\n')
fprintf('This script uses a ray-casting algorithm to estimate the visual range of our robot.\n')
fprintf('Press any key to continue.\n')
pause

% add dependencies
addpath('../kinematics')
addpath('../path-planning')`
addpath('../utils')
addpath('../utils/ray-casting')
addpath('../utils/stlTools')
addpath('../utils/visibility')
addpath('../../anatomical-models')


% define the robot's range of motion
maxfprintflacement = 1; % [mm]
maxRotation     = 2*pi; % [rad]
maxAdvancement  = 10; % [mm]

% Load cavity model
path = fullfile('..', '..', 'anatomical-models', 'synthetic-model.stl');
[vertices, faces, ~, ~] = stlRead(path);
earModel.vertices = vertices;
earModel.faces = faces;

% Calculate the base transform for the robot
t = [30 8 10];
R = [0 0 -1; 0 1 0; 1 0 0];
T = [R t'; 0 0 0 1];
earModel.baseTransform = T;

% Create a robot
alpha = pi;
cutouts = [];
cutouts.w = [1 1 1 1 1 1];
cutouts.u = [1 1 1 1 1 1];
cutouts.h = [1 1 1 1 1 1];
cutouts.alpha = [0 0 0 alpha 0 0];
robot = Wrist(1.6, 1.85, 6, cutouts);

fprintf('Running RRT...\n')

[~,qList,pList,aList] = rrt(robot, ...
    [maxfprintflacement maxRotation maxAdvancement], ...
    earModel, ...
    nPoints);

fprintf(['RRT execution complete. Total sampled points: ' num2str(size(pList,2)) ' \n\n']);

% Load cavity model (finer mesh this time)
path = fullfile('..', '..', 'anatomical-models', 'synthetic-model-finer.stl');
[vertices, faces, ~, ~] = stlRead(path);
earModel.vertices = vertices;
earModel.faces = faces;

figure

% Visualize the visual range for each pose of the robot
ii = 1;
seenMap = visibilitymap(pList(:,ii), aList(:,ii), earModel);
h1 = stlPlot(earModel.vertices, earModel.faces, 'Ray Casting test.', seenMap);
hold on

robotPhysicalModel = robot.makePhysicalModel(qList(:,1), T);
h2 = surf(robotPhysicalModel.surface.X, ...
    robotPhysicalModel.surface.Y, ...
    robotPhysicalModel.surface.Z, ...
    'FaceColor','blue');

axis equal

%for ii = 1 : size(pList, 2)
while true
    seenMap = visibilitymap(pList(:,ii), aList(:,ii), earModel);
    robotPhysicalModel = robot.makePhysicalModel(qList(:,ii), T);
    
    colorMap = ones(length(vertices), 1);
    colorMap(logical(seenMap)) = 5;
    h1.FaceVertexCData = colorMap;
    
    h2.XData = robotPhysicalModel.surface.X;
    h2.YData = robotPhysicalModel.surface.Y;
    h2.ZData = robotPhysicalModel.surface.Z;
    title(['Pose ' num2str(ii) ' of ' num2str(size(pList, 2))]);
    
    fprintf('Press "n" to move forward or "p" to move back.\n')
    fprintf('Press any other key to stop testing and generate the reachable workspace.\n\n')
    
    while ~waitforbuttonpress, end
    k = get(gcf, 'CurrentCharacter');
    
    switch k
        case 'p'
            ii = ii - 1;
            if ii < 1, ii = 1; end
        case 'n'
            ii = ii + 1;
            if ii > size(pList, 2), ii = size(pList, 2); end
        otherwise
            break
    end
end

fprintf('Testing complete.\n')