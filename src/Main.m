%% Robot endoscope features.
clc, clear, close all
addpath('kinematics')
addpath('path-planning')
addpath('utils')

cutouts(1).w = 1;
cutouts(1).u = 1;
cutouts(1).h = 1;
cutouts(1).alpha = 0;

robot = Wrist(1.6, 1.85, 3, cutouts);
%teach(robot, [0.2, 0, 0]);

[qList,pList,aList] = rrt(robot, [1 1 10]);

figure, axis equal, grid on
scatter3(qList(1,:), qList(2,:), qList(3,:));
xlabel('Pull-wire displacement [mm]');
ylabel('Axial rotation [rad]');
zlabel('Axial translation [mm]');
title('Configurations generated by RRT');

figure, axis equal, grid on
scatter3(pList(1,:), pList(2,:), pList(3,:));
xlabel('X [mm]'), ylabel('Y [mm]'), zlabel('Z [mm]');
title('Reachable points in the task space');