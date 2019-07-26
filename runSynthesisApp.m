function runSynthesisApp()
%% Main function for Notched Tube Designer App
% Created by: Floris van Rossum, 5/28/2019
% COgnitive MEdical Technology Robotics Laboratory (COMET Robotics Lab)
% Worcester Polytechnic Institute

%% Program Configuration
% Close all previous windows and add folders, sub-folders to path

addpath('src');
addpath('src/utils');
addpath('src/utils/stlTools/');
addpath('src/kinematics'); 
addpath('src/gui');
addpath("anatomical-models/*");
addpath("src/synthesis");

app = synthesisController();

end