%% Script to test the robot kinematics
%clc, clear, close all
addpath('kinematics')
addpath('path-planning')
addpath('utils')

n = 8;
w = 1.36;
u = 0.92;
h = 0.17;

cutouts.w = 1.36 * ones(1,n) * 1e-3; % [m]
cutouts.u = [u * ones(1,n-1) * 1e-3, 4.5 * 1e-3]; % [m]
cutouts.h = h * ones(1,n) * 1e-3; % [m]
cutouts.alpha = zeros(1,n);
cutouts.alpha = zeros(1,n);
robot = Wrist(1.4e-3, 1.6e-3, n, cutouts);

configuration = [sum(cutouts.h) 0 2e-3];

robot.fwkine(configuration, eye(4));
robotModel = robot.makePhysicalModel();
P = robot.pose(:,end);
T = robot.transformations;

X = P(1,:);
Y = P(2,:);
Z = P(3,:);

% figure
% scatter3(X, Y, Z, 100, 'r', 'filled');
% hold on, axis equal
% 
% plot3(X, Y, Z, 'k', 'LineWidth', 2.5);
% plot3(robotModel.backbone(1,:), ...
%       robotModel.backbone(2,:), ...
%       robotModel.backbone(3,:), ...
%       'LineWidth', 2.5);
%   
% xlabel('X[mm]')
% ylabel('Y[mm]')
% zlabel('Z[mm]')
% 
% for ii = 1 : size(T,3)
%     triad('Matrix', T(:,:,ii), 'linewidth', 2.5, 'scale', 1e-4);    
% end

figure
% scatter3(X, Y, Z, 100, 'r', 'filled');

% plot3(X, Y, Z, 'k', 'LineWidth', 2.5);
% triad('Matrix', eye(4), 'linewidth', 2.5, 'scale', 1e-4);
% triad('Matrix', T(:,:,end), 'linewidth', 2.5,  'scale', 1e-4);

X = robotModel.surface.X;
Y = robotModel.surface.Y;
Z = robotModel.surface.Z;
surf(X, Y, Z, 'FaceColor','blue');
axis equal
xlabel('X[m]')
ylabel('Y[m]')
zlabel('Z[m]')
zlim([0 0.012]);
xlim([-0.0008 7e-3]);

% X = robotModel.surface.X;
% Y = robotModel.surface.Y;
% Z = robotModel.surface.Z;
% surf(X, Y, Z, 'FaceColor','blue');

view(0,0);