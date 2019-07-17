function runSynthesisApp()
%% Main function for Notched Tube Designer App
% Created by: Floris van Rossum, 5/28/2019
% COgnitive MEdical Technology Robotics Laboratory (COMET Robotics Lab)
% Worcester Polytechnic Institute

%% Program Configuration
% Close all previous windows and add folders, sub-folders to path

addpath('utils');
addpath('utils/stlTools/');
addpath('kinematics'); 
addpath('gui');
addpath("anatomical-models");

app = synthesisController();

end