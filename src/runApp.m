%% Main function for Notched Tube Designer App
% Created by: Floris van Rossum, 5/28/2019
% COgnitive MEdical Technology and Robotics Laboratory (COMET Robotics Lab)
% Worcester Polytechnic Institute

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
app = NotchedDesigner();
var = 0.7;
back = 0;
while ~app.stopFlag
    disp(var);
    if back
        var = var - 0.1;
    else
        var = var + 0.1;
    end
    
    if var > 1.7
        back = 1;
    elseif var < 0.7
        back = 0;
    end
    showRobot(app, var);
    pause(0.2);
end

disp("Stopped");
