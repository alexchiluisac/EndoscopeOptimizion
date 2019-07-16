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
m = 2;
partitionedCurve = partitioncurve(curve, m, 'plot', true);

%% 3. Synthesize a wrist that approximates the curve 
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
n = 5 * ones(1, m);

% height of the notches and of the uncut sections
h = zeros(1, m);
u = zeros(1, m);
T = eye(4);

figure, hold on

for ii = 1 : m
    h(ii) = kappa(ii) * l(ii) * (ro+ybar)/n(ii);
    u(ii) = (l(ii) - kappa(ii)*l(ii)*ro)/n(ii);
    
    cutouts{ii}.w = w*10^3 .* ones(1,n(ii));
    cutouts{ii}.u = u(ii)*10^3 .* ones(1,n(ii));
    cutouts{ii}.h = h(ii)*10^3 .* ones(1,n(ii));
    cutouts{ii}.alpha = zeros(1,n(ii));
    
    %configuration{ii} = [h(ii)*10^3, 0, 0];
    configuration{ii} = [0.01, 0, 0];
    
    robot{ii} = Wrist(ID*10^3, OD*10^3, n(ii), cutouts{ii});
    robot{ii}.fwkine(configuration{ii}, T);
    Treal = robot{ii}.transformations(:,:,end);
    Treal(1:3,end) = Treal(1:3,end) / 1000;
    T = T * Treal;
    
    robotModel{ii} = robot{ii}.makePhysicalModel();
    X = robotModel{ii}.backbone(1,:) * 10^-3;
    Y = robotModel{ii}.backbone(2,:) * 10^-3;
    Z = robotModel{ii}.backbone(3,:) * 10^-3;
    scatter3(X, Y, Z, 100, col(7,:), 'filled');
    
    X = robotModel{ii}.surface.X * 10^-3;
    Y = robotModel{ii}.surface.Y * 10^-3;
    Z = robotModel{ii}.surface.Z * 10^-3;
    surf(X, Y, Z, 'FaceColor',col(6,:));
end

title('Wrist synthesis');
xlabel('X [m]'), ylabel('Y [m]'), zlabel('Z [m]')
set(gca,'FontSize',16);
axis equal, axis tight, grid on
view(0.26, 20.5)

return
% Display the wrist
% X = robot.pose(1,:) * 10^-3;
% Y = robot.pose(2,:) * 10^-3;
% Z = robot.pose(3,:) * 10^-3;
% 
% scatter3(X, Y, Z, 100, 'r', 'filled');
%hold on, axis equal
% 
% X = robotModel1.backbone(1,:) * 10^-3;
% Y = robotModel1.backbone(2,:) * 10^-3;
% Z = robotModel1.backbone(3,:) * 10^-3;
% scatter3(X, Y, Z, 100, col(7,:), 'filled');
% 
% X = robotModel2.backbone(1,:) * 10^-3;
% Y = robotModel2.backbone(2,:) * 10^-3;
% Z = robotModel2.backbone(3,:) * 10^-3;
% scatter3(X, Y, Z, 100, col(7,:), 'filled');
% 
% X = robotModel3.backbone(1,:) * 10^-3;
% Y = robotModel3.backbone(2,:) * 10^-3;
% Z = robotModel3.backbone(3,:) * 10^-3;
% scatter3(X, Y, Z, 100, col(7,:), 'filled');
% 
% axis equal
% pause
% 
% X = robotModel1.surface.X * 10^-3;
% Y = robotModel1.surface.Y * 10^-3;
% Z = robotModel1.surface.Z * 10^-3;
% surf(X, Y, Z, 'FaceColor',col(6,:));
% 
% X = robotModel2.surface.X * 10^-3;
% Y = robotModel2.surface.Y * 10^-3;
% Z = robotModel2.surface.Z * 10^-3;
% surf(X, Y, Z, 'FaceColor',col(6,:));
% 
% X = robotModel3.surface.X * 10^-3;
% Y = robotModel3.surface.Y * 10^-3;
% Z = robotModel3.surface.Z * 10^-3;
% surf(X, Y, Z, 'FaceColor',col(6,:));