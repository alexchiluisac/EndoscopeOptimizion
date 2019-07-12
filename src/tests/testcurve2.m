%% Script to test the robot kinematics
clc, clear, close all
addpath('kinematics')
addpath('path-planning')
addpath('utils')

col = distinguishable_colors(10);

% First, let's generate a constant curvature arc
l = 1 * 10^-3; % [m] total arc length
k = 500;         % [m^-1] curvature
r = 1/k;       % [m] radius of curvature

theta = 0:l*k/20:l*k;

arc = r .* [(1-cos(theta)); 
            zeros(1, length(theta));
            sin(theta)];

figure
triad('scale', 10^-3, 'linewidth', 2.5)
hold on
plot3(arc(1,:), arc(2,:), arc(3,:),'MarkerEdgeColor', col(1,:), 'LineWidth', 2.5);   
grid on, axis equal
xlabel('X [m]'), ylabel('Y [m]'), zlabel('Z [m]');
view(136, 30);

OD = 1.85 * 10^-3; % [m] tube outer diameter
ID = 1.60 * 10^-3; % [m] tube inner diameter
ro = OD/2;         % [m] tube outer radius
ri = ID/2;         % [m] tube inner radius

% Now let's synthesize the wrist
w = .85 * OD; % [m]
d = w - ro;   % [m]
phio = 2 * acos(d / ro); % [rad]
phii = 2 * acos(d / ri); % [rad]
ybaro = (4 * ro * (sin(0.5 * phio)) ^ 3)/ (3 * (phio - sin(phio)));
ybari = (4 * ri * (sin(0.5 * phii)) ^ 3)/ (3 * (phio - sin(phii)));
Ao = ( (ro ^ 2) * ( phio - sin(phio))) / 2;
Ai = ( (ri ^ 2) * ( phii - sin(phii))) / 2;
ybar = (ybaro * Ao - ybari * Ai) / (Ao - Ai);
    
% number of notches
n = 2;

% height of the notches
h = k*l*(ro+ybar)/n;

% length of the uncut section
%u = 2/k * sin(k*l/4);
u = (l - k*l*ro)/n;

cutouts.w = w*10^3 .* ones(1,n);
cutouts.u = u*10^3 .* ones(1,n);
cutouts.h = h*10^3 .* ones(1,n);
cutouts.alpha = zeros(1,n);

configuration = [0.42, 0, 0];

robot = Wrist(ID*10^3, OD*10^3, n, cutouts);
robot.fwkine(configuration, eye(4));

% Display the wrist
% X = robot.pose(1,:) * 10^-3;
% Y = robot.pose(2,:) * 10^-3;
% Z = robot.pose(3,:) * 10^-3;
% 
% scatter3(X, Y, Z, 100, 'r', 'filled');
%hold on, axis equal

robotModel = robot.makePhysicalModel();
% 
% X = robotModel.surface.X * 10^-3;
% Y = robotModel.surface.Y * 10^-3;
% Z = robotModel.surface.Z * 10^-3;
% surf(X, Y, Z, 'FaceColor',col(6,:));

X = robotModel.backbone(1,:) * 10^-3;
Y = robotModel.backbone(2,:) * 10^-3;
Z = robotModel.backbone(3,:) * 10^-3;
scatter3(X, Y, Z, 100, col(7,:), 'filled');