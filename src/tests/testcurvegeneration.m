%% Script to test the wrist synthesis algorithm
clc, clear, close all

addpath('../synthesis')
addpath('../kinematics')
addpath('../utils')

col = distinguishable_colors(10);

%% 1. Generate a curve

% Arc Length in [m]
arcLength = 20e-3; 

% Curvature Profile [1/m]
%k = @(s,arcLength) 100 .* ones(1,length(s));
k = @(s,arcLength) 100 .* s/arcLength;

% Torsional Profile
% tau = @(s,arcLength) 0 * s/arcLength; 

curve = {};

T = eye(4);

% Create the curve with the MAKECURVE function
curve{1} = makecurve(arcLength, k);
R = [curve{1}.n(:,end) curve{1}.b(:,end) curve{1}.t(:,end)];
t = curve{1}.arc(:,end);
T2 = [R t; 0 0 0 1];

curve{2} = makecurve(arcLength, k, 'transform', T2);
R = [curve{2}.n(:,end) curve{2}.b(:,end) curve{2}.t(:,end)];
t = curve{2}.arc(:,end);
T3 = [R t; 0 0 0 1];

curve{3} = makecurve(arcLength, k, 'transform', T3);

figure
scatter3(curve{1}.arc(1,:), curve{1}.arc(2,:), curve{1}.arc(3,:),'MarkerEdgeColor', col(5,:), 'LineWidth', 2.5);
hold on
scatter3(curve{2}.arc(1,:), curve{2}.arc(2,:), curve{2}.arc(3,:),'MarkerEdgeColor', col(6,:), 'LineWidth', 2.5);
scatter3(curve{3}.arc(1,:), curve{3}.arc(2,:), curve{3}.arc(3,:),'MarkerEdgeColor', col(7,:), 'LineWidth', 2.5);
axis equal, grid on
%xlim([-2e-3 2e-3]), ylim([-2e-3 2e-3]), zlim([0 5e-3]);
xlabel('X [m]'), ylabel('Y [m]'), zlabel('Z [m]'),
view(0.26, 20.5)
set(gca,'FontSize',16);
title('Target Curve');

h = triad('scale', 1e-3/2, 'linewidth', 2.5);
triad('scale', 1e-2, 'linewidth', 2.5, 'matrix', curve{1}.nextTransform);
triad('scale', 1e-2, 'linewidth', 2.5, 'matrix', curve{2}.nextTransform);
triad('scale', 1e-2, 'linewidth', 2.5, 'matrix', curve{3}.nextTransform);