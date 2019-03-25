function seenMap = estimateVisibleSurface(alpha)
%% Continuum robot optimization for a cavity exploration task
% add dependencies

% define the robot's range of motion
maxDisplacement = 1; % [mm]
maxRotation     = 2*pi; % [rad]
maxAdvancement  = 10; % [mm]

% Load cavity model
path = fullfile('..', 'anatomical-models', 'synthetic-model.stl');
[vertices, faces, ~, ~] = stlRead(path);
earModel.vertices = vertices;
earModel.faces = faces;

% Calculate the base transform for the robot
t = [30 8 10];
R = [0 0 -1; 0 1 0; 1 0 0];
T = [R t'; 0 0 0 1];
earModel.baseTransform = T;

% Create a robot
%alpha = 0;
cutouts = [];
cutouts.w = [1 1 1 1];
cutouts.u = [1 1 1 1];
cutouts.h = [1 1 1 1];
cutouts.alpha = [0 0 alpha 0];

robot = Wrist(1.6, 1.85, 4, cutouts);

% Run RRT to estimate the reachable workspace of the robot
nPoints = 100;

[~,~,pList,aList] = rrt(robot, ...
    [maxDisplacement maxRotation maxAdvancement], ...
    earModel, ...
    nPoints);


% Estimate the reachable workspace
%shrinkFactor = 1;
%[k,v] = boundary(pList(1,:)', pList(2,:)', pList(3,:)', shrinkFactor);

%disp(['Reachable volume: ' num2str(v) ' mm3']);

% Re-load the cavity model (a finer mesh this time)
path = fullfile('..', 'anatomical-models', 'synthetic-model-finer.stl');
[vertices, faces, ~, ~] = stlRead(path);
earModel.vertices = vertices;
earModel.faces = faces;

% Visualize the visual range for each pose of the robot
seenMap = zeros(1, size(earModel.vertices, 1));

for ii = 1 : size(pList, 2)
    seenMap = seenMap + visiblesurface(pList(:,ii), aList(:,ii), earModel);
end

seenMap(seenMap > 1) = 1;

end
