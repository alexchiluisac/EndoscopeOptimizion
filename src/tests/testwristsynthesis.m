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

% Create the curve with the MAKECURVE function
curve = makecurve(arcLength, k, 'plot', true);

%% 2. Partition the curve into m sections
m = 5;
partitionedCurve = partitioncurve(curve, m, 'plot', true);

%% 3. Synthesize a wrist that bends like the curve
OD = 1.85 * 10^-3; % [m] tube outer diameter
ID = 1.60 * 10^-3; % [m] tube inner diameter
ro = OD/2;         % [m] tube outer radius
ri = ID/2;         % [m] tube inner radius

l = diff(partitionedCurve.l);
kappa = partitionedCurve.averageKappa;

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
n = 1 * ones(1, m);

% height of the notches and of the uncut sections
h = zeros(1, m);
u = zeros(1, m);
T = eye(4);

cutouts.w = [];
cutouts.u = [];
cutouts.h = [];
cutouts.alpha = [];

figure, hold on

for ii = 1 : m
    h(ii) = kappa(ii) * l(ii) * (ro+ybar)/n(ii);
    u(ii) = (l(ii) - kappa(ii)*l(ii)*ro)/n(ii);
    
    cutouts.w = [cutouts.w w*10^3 .* ones(1,n(ii))];
    cutouts.u = [cutouts.u u(ii)*10^3 .* ones(1,n(ii))];
    cutouts.h = [cutouts.h h(ii)*10^3 .* ones(1,n(ii))];
    cutouts.alpha = [cutouts.alpha zeros(1,n(ii))];
end

%configuration{ii} = [h(ii)*10^3, 0, 0];
configuration = [0.12, 0, 0];

robot = Wrist(ID*10^3, OD*10^3, sum(n), cutouts);
robot.fwkine(configuration, T);
%Treal = robot{ii}.transformations(:,:,end)
%T = T * Treal

robotModel = robot.makePhysicalModel();
X = robotModel.backbone(1,:) * 10^-3;
Y = robotModel.backbone(2,:) * 10^-3;
Z = robotModel.backbone(3,:) * 10^-3;
scatter3(X, Y, Z, 100, col(7,:), 'filled');
    
X = robotModel.surface.X * 10^-3;
Y = robotModel.surface.Y * 10^-3;
Z = robotModel.surface.Z * 10^-3;
surf(X, Y, Z, 'FaceColor',col(6,:));

title('Wrist synthesis');
xlabel('X [m]'), ylabel('Y [m]'), zlabel('Z [m]')
set(gca,'FontSize',16);
axis equal, axis tight, grid on
view(0.26, 20.5)