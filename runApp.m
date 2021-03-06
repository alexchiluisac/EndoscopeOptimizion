function runApp()
%% Main function for Notched Tube Designer App
% Created by: Floris van Rossum, 5/28/2019
% COgnitive MEdical Technology Robotics Laboratory (COMET Robotics Lab)
% Worcester Polytechnic Institute

%% Program Configuration
% Close all previous windows and add folders, sub-folders to path
clc; close all; clear classes;
addpath("src");
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
addpath("anatomical-models");
%% Robot Settings

% Defining cut-outs
% Width of cut: 85% of the OD
cutouts.w = [1.275 1.275 1.275 1.275];
cutouts.u = [1 1 1 1];
cutouts.h = [1 1 1 1];
cutouts.alpha = [0 0 pi/2 0];

id = 1.3; % Inner-diameter of the robot (mm)
od = 1.5; % Outer-diameter of the robot (mm)
nCutouts = 4; % Number of cutouts in the robot

%% Start-up

% Create the controller object, start the interface
controller = appController(id, od, nCutouts, cutouts);

end
