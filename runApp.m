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

%% Robot Settings

% Defining cut-outs
cutouts.w = [1 1 1 1];
cutouts.u = [1 1 1 1];
cutouts.h = [1 1 1 1];
cutouts.alpha = [0 0 pi/2 0];

% Create the controller object, start the interface
controller = appController(1.65, 1.85, 4, cutouts);
