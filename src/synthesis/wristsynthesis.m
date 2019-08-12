%% Script to synthesize a wrist
% This script synthesizes the design parameters of a Nitinol-based
% notched tube wrist. The synthesis is performed in such a way that the
% wrist, once at full bending, articulates in a prescribed manner, which is
% described by a 3D curve in space.

% The output of the script is an object of class WRIST. The attribute
% `cutouts' of the object contains the design parameters. See the
% documentation of the WRIST class for a description of the data fields and
% their meaning.

clc, clear, close all
addpath('kinematics')
addpath('utils')

col = distinguishable_colors(10);

%% Inputs/parameters of the algorithm to be defined below:

% === Curve description ===
% Arc Length (m)
arcLength = 5*1e-3; % [m]

% Curvature profile (1/m)
%k = @(s,arcLength) 150 .* s/arcLength; % increasing curvature
%k = @(s,arcLength) 444 .* ones(1,length(s)); % constant curvature
k = @(s,arcLength) 500 .* ones(1,length(s));

% Torsional Profile
tau = @(s,arcLength) 1000 .* ones(1,length(s)); 


% === Algorithm Parameters ===
% Number of sections in which the original curve should be partitioned:
m = 5; % this parameter is important in curves with varying curvature profiles - for a constant curvature arc, m can simply be equal to 1

% Number of notches for each section
n = 2 * ones(1, m); % increasing the number of notches will make the wrist better approximate the original curve

%% === THERE SHOULD BE NO NEED TO CHANGE THE CODE BELOW ===
% 1. Create the curve with the MAKECURVE function
curve = makecurve(arcLength, k, tau, 'plot', true);

% 2. Partition the curve into m sections
partitionedCurve = partitioncurve(curve, m, 'plot', true);

% 3. Synthesize a wrist that bends like the curve
OD = 1.40 * 10^-3; % [m] tube outer diameter
ID = 1.20 * 10^-3; % [m] tube inner diameter
ro = OD/2;         % [m] tube outer radius
ri = ID/2;         % [m] tube inner radius

l = diff(partitionedCurve.l);
kappa = partitionedCurve.averageKappa;
tau = partitionedCurve.averageTau;

w = .85 * OD; % [m]
d = w - ro;   % [m]
phio = 2 * acos(d / ro); % [rad]
phii = 2 * acos(d / ri); % [rad]
ybaro = (4 * ro * (sin(0.5 * phio)) ^ 3)/ (3 * (phio - sin(phio)));
ybari = (4 * ri * (sin(0.5 * phii)) ^ 3)/ (3 * (phio - sin(phii)));
Ao = ( (ro ^ 2) * ( phio - sin(phio))) / 2;
Ai = ( (ri ^ 2) * ( phii - sin(phii))) / 2;
ybar = (ybaro * Ao - ybari * Ai) / (Ao - Ai);
    
% height of the notches and of the uncut sections
h = zeros(1, m);
u = zeros(1, m);
alpha = zeros(1, m);
T = eye(4);

cutouts.w = [];
cutouts.u = [];
cutouts.h = [];
cutouts.alpha = [];
l = diff(partitionedCurve.l);

figure, hold on

for ii = 1 : m
    
    h(ii) = kappa(ii) * l(ii) * (ro+ybar)/n(ii);
    u(ii) = (l(ii) - kappa(ii)*l(ii)*ro)/n(ii);
    alpha(ii) = tau(ii) * l(ii);
    
    cutouts.w = [cutouts.w w*10^3 .* ones(1,n(ii))];
    cutouts.u = [cutouts.u u(ii)*10^3 .* ones(1,n(ii))];
    cutouts.h = [cutouts.h h(ii)*10^3 .* ones(1,n(ii))];
    cutouts.alpha = [cutouts.alpha alpha(ii) zeros(1,n(ii)-1)];
end

configuration = [sum(cutouts.h), 0, 0];

robot = Wrist(ID*10^3, OD*10^3, sum(n), cutouts);
robot.fwkine(configuration, T);

robotModel = robot.makePhysicalModel();    
X = robotModel.surface.X * 10^-3;
Y = robotModel.surface.Y * 10^-3;
Z = robotModel.surface.Z * 10^-3;
surf(X, Y, Z, 'FaceColor',col(6,:));

X = robotModel.backbone(1,:) * 10^-3;
Y = robotModel.backbone(2,:) * 10^-3;
Z = robotModel.backbone(3,:) * 10^-3;
scatter3(X, Y, Z, 100, col(7,:), 'filled');

title('Wrist synthesis');
xlabel('X [m]'), ylabel('Y [m]'), zlabel('Z [m]')
set(gca,'FontSize',16);
axis equal, axis tight, grid on
view(0.26, 20.5)

figure
scatter3(curve.arc(1,:), curve.arc(2,:), curve.arc(3,:),'MarkerEdgeColor', col(5,:), 'LineWidth', 2.5);
hold on
triad('scale', 1e-3/2, 'linewidth', 2.5);
scatter3(X, Y, Z, 100, col(7,:), 'filled');
xlabel('X [m]'), ylabel('Y [m]'), zlabel('Z [m]')
title('Wrist pose vs target curve');
set(gca,'FontSize',16);
axis equal, axis tight, grid on
view(0.26, 20.5)