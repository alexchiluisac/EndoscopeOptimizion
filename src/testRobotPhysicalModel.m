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

maxDisplacement = 1; % [mm]
maxRotation     = 2*pi; % [rad]
maxAdvancement  = 10; % [mm]



[X, Y, Z] = gencyl(link(1:3,:), 1.8 / 2 * ones(1,size(link,2)));

%[qListNormalized,qList,pList,aList] = rrt(robot, ...
%    [maxDisplacement maxRotation maxAdvancement]);