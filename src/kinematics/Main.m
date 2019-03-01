%% Robot endoscope features.
clc, clear, close all
addpath('../utils')

cutouts(1).w = 1;
cutouts(1).u = 1;
cutouts(1).h = 1;
cutouts(1).alpha = 0;

robot = Wrist(1.6, 1.85, 3, cutouts);
teach(robot, [0.2, 0, 0]);