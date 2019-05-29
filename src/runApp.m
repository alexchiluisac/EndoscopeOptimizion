%% Main function for Notched Tube Designer App
% Created by: Floris van Rossum, 5/28/2019
% COgnitive MEdical Technology and Robotics Laboratory (COMET Robotics Lab)
% Worcester Polytechnic Institute

%% Program Configuration
% Close all previous windows and add folders, sub-folders to path
clc; close all; clear all;
addpath(".");
addpath('cost-functions'); 
addpath('utils');
addpath('utils/stlTools/');
addpath('utils/visibility/');
addpath('kinematics');
addpath('path-planning');
addpath('utils/ray-casting/');
addpath('gui');
addpath('gui/images');

pause('on');

%% Robot Settings

% Defining cut-outs
cutouts.w = [1 1 1 1];
cutouts.u = [1 1 1 1];
cutouts.h = [1 1 1 1];
cutouts.alpha = [0 0 pi/2 0];
controller = appController(1.65, 1.85, 4, cutouts);

%% Execution
while ~controller.app.stopFlag
    controller.update();
    pause(0.1);
end
