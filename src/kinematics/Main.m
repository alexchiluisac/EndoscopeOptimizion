%% Robot endoscope features.
clc; clear; close all;
addpath('../utils')
cutouts(1).w = 1;
cutouts(1).u = 1;
cutouts(1).h = 1;
cutouts(1).alpha = 0;
r = Robot(1.6,1.85,3,cutouts)
% T = r.fwkine([0.5, 0, 0])
teach(r,[0.5, 0, 0])