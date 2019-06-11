%% Main function for Notched Tube Designer App
% Created by: Floris van Rossum, 5/28/2019
% COgnitive MEdical Technology and Robotics Laboratory (COMET Robotics Lab)
% Worcester Polytechnic Institute

%% Program Configuration
% Close all previous windows and add folders, sub-folders to path
clc; close all; clear classes;
addpath("src");
addpath("anatomical-models");
addpath('src/cost-functions'); 
addpath('src/utils');
addpath('src/utils/stlTools/');
addpath('src/utils/visibility/');
addpath('src/kinematics'); 
addpath('src/path-planning');
addpath('src/utils/ray-casting/');
addpath('src/gui');
addpath('src/gui/images');
addpath('src/collision/MatlabCode');

%% Robot Settings

% Defining cut-outs
% Width of cut: 85% of the OD
cutouts.w = 1.275 * ones(4);
cutouts.u = ones(4);
cutouts.h = ones(4);
cutouts.alpha = [0 0 pi/2 0];

% Create the controller object, start the interface
controller = appController(1.3, 1.5, 4, cutouts);
