%% Robot endoscope features.
clc, clear, close all
addpath('kinematics')
addpath('path-planning')
addpath('utils')
addpath('../anatomical-models')

cutouts.w = [1 1 1 1];
cutouts.u = [1 1 1 1];
cutouts.h = [1 1 1 1];
cutouts.alpha = [0 0 pi 0];

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

alpha = 0 : pi/4 : pi;

for a = alpha
    cutouts.alpha = [0 0 a 0];
    % Create the robot
    robot = Wrist(1.6, 1.85, 4, cutouts);
    
    [qListNormalized,qList,pList,aList] = rrt(robot, ...
        [maxDisplacement maxRotation maxAdvancement], ...
        earModel);
    
    [k,v] = boundary(pList(1,:)', pList(2,:)', pList(3,:)', 0.5);
    reachableVolume(i,1) = v;
    i = i + 1;
    
end

% Plot objective function
figure
scatter(alpha * 180 / pi, reachableVolume)
%axis equal
grid on
xlabel('Angle [Degree]'), ylabel('Reachable Volume [mm^3]');
xlim([alpha(1) alpha(end)]  * 180 / pi)
ylim([0 max(reachableVolume)])