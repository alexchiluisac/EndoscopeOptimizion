%% Script to test the robot differential kinematics
clc, clear, close all
addpath('kinematics')
addpath('path-planning')
addpath('utils')

c = distinguishable_colors(10);

cutouts.w = [1 1];
cutouts.u = [1 1];
cutouts.h = [1 1];
cutouts.alpha = [0 0];

%configuration = [0.6, pi/6, 2];
configuration = [0, 0, 0];

robot = Wrist(1.6, 1.85, 2, cutouts);
robot.fwkine(configuration, eye(4));
robotModel = robot.makePhysicalModel();

pTarget = [1.5443 0.8916 6]';

figure
scatter3(pTarget(1), pTarget(2), pTarget(3), 100, c(7,:), 'filled');
text(pTarget(1)+.5, pTarget(2)+.5, pTarget(3)+.5, 'Target Point', 'Fontsize', 10, 'Color', c(7,:));
title('Numerical Inverse Kinematics of a Notched-tube Wrist');
xlabel('X[mm]')
ylabel('Y[mm]')
zlabel('Z[mm]')
hold on, axis equal
view(-192.14, 6.56);

triad('Matrix', eye(4), 'linewidth', 2.5);
robotRef = triad('Matrix', robot.transformations(:,:,end), 'linewidth', 2.5);

X = robotModel.surface.X;
Y = robotModel.surface.Y;
Z = robotModel.surface.Z;
h = surf(X, Y, Z, 'FaceColor',c(6,:));
hold on

qCurrent = configuration;
pTarget = pTarget * 10^-3;

while true
    % calculate the current location
    robot.fwkine(qCurrent, eye(4));
    pCurrent = robot.pose(1:3,end) * 10^-3;

    % plot
    try
        robotModel = robot.makePhysicalModel();
        h.XData = robotModel.surface.X;
        h.YData = robotModel.surface.Y;
        h.ZData = robotModel.surface.Z;
        robotRef.Matrix = robot.transformations(:,:,end);
        drawnow
    catch e
        disp(e);
    end
    
    % calculate the difference between current and target location
    err = norm(pTarget - pCurrent);
    err * 10^3
    
    % if the difference < epsilon, return
    if err < 1e-6, break; end
    
    % else, update the "joint" variables using the inverse of the
    % jacobian
    [~,Jp] = robot.jacob0(qCurrent)
    %Jp = J(1:3,:) - skew(pCurrent) * J(4:6,:)
    Jpinv = pinv(Jp);
    
    K = diag([100 0.1 100]);
    deltaQ = K * Jpinv * (pTarget - pCurrent);
%     deltaQ(1) = deltaQ(1) / 1000;
%     deltaQ(3) = deltaQ(3) / 1000;
    qCurrent = qCurrent + deltaQ'
    
    
end
