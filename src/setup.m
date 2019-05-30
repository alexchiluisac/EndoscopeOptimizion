%% Setup.m
% Run to load all sub-folders and clear previous results
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
addpath('+arduinoioaddons');
addpath('+arduinoioaddons/+Nunchuk');
addpath('+arduinoioaddons/+Nunchuk/src');