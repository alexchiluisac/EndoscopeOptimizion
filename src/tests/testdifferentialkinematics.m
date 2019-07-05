%% Script to test the robot kinematics
clc, clear, close all
addpath('kinematics')
addpath('path-planning')
addpath('utils')

cutouts.w = [1];
cutouts.u = [1];
cutouts.h = [1];
cutouts.alpha = [0];

configuration = [0, 0, 2];

robot = Wrist(1.6, 1.85, 1, cutouts);
robot.fwkine(configuration, eye(4));
%[Jrobot,Jp] = robot.jacob0(configuration)

robotModel = robot.makePhysicalModel();

% X = robot.pose(1,:);
% Y = robot.pose(2,:);
% Z = robot.pose(3,:);

% figure
% scatter3(X, Y, Z, 100, 'r', 'filled');
% hold on, axis equal
% 
% plot3(X, Y, Z, 'k', 'LineWidth', 2.5);
% xlabel('X[mm]')
% ylabel('Y[mm]')
% zlabel('Z[mm]')
% 
% for ii = 1 : size(robot.transformations,3)
%     triad('Matrix', robot.transformations(:,:,ii), 'linewidth', 2.5);    
% end

figure
scatter3(robotModel.backbone(1,:), ...
         robotModel.backbone(2,:), ...
         robotModel.backbone(3,:), 100, 'r', 'filled');
hold on, axis equal

%plot3(X, Y, Z, 'k', 'LineWidth', 2.5);
xlabel('X[mm]')
ylabel('Y[mm]')
zlabel('Z[mm]')

triad('Matrix', eye(4), 'linewidth', 2.5);
robotRef = triad('Matrix', robot.transformations(:,:,end), 'linewidth', 2.5);

X = robotModel.surface.X;
Y = robotModel.surface.Y;
Z = robotModel.surface.Z;
h = surf(X, Y, Z, 'FaceColor','yellow');
hold on

qCurrent = configuration;

pTarget = [1 1 10]' * 10^-3;

while true
    % calculate the current location
    robot.fwkine(qCurrent, eye(4));
    pCurrent = robot.pose(1:3,end) * 10^-3;

    % plot
%     robotModel = robot.makePhysicalModel();
%     h.XData = robotModel.surface.X;
%     h.YData = robotModel.surface.Y;
%     h.ZData = robotModel.surface.Z;
%     robotRef.Matrix = robot.transformations(:,:,end);
%     drawnow
    
    % calculate the difference between current and target location
    err = norm(pTarget - pCurrent)
    err * 10^3
    
    % if the difference < epsilon, return
    if err < 1e-6, break; end
    
    % else, update the "joint" variables using the inverse of the
    % jacobian
    [~,Jp] = robot.jacob0(qCurrent);
    %Jp = J(1:3,:) - skew(pCurrent) * J(4:6,:)
    Jpinv = pinv(Jp);
    
    K = diag([1 1 1]);
    deltaQ = K * Jpinv * (pTarget - pCurrent);
%     deltaQ(1) = deltaQ(1) / 1000;
%     deltaQ(3) = deltaQ(3) / 1000;
    qCurrent = qCurrent + deltaQ';
    
    
end
