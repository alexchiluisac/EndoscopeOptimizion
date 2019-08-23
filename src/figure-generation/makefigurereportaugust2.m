%close all
clear, clc

addpath('kinematics')
addpath('path-planning')
addpath('utils')

load 10-simulation.mat

figure('units','normalized','outerposition', [0 0 1 1])
hold on

pathStl = fullfile('..', 'anatomical-models', 'atlas', 'me.stl');
[vertices, faces, ~, ~] = stlRead(pathStl);
earModel.vertices = vertices;
earModel.faces = faces;
stlPlot(earModel.vertices, earModel.faces, 'Ear Model');
stlPlot(osModel.vertices, osModel.faces, 'Ear Model', 10);
%view([17.8 30.2]);

scatter3(pList(1,:), pList(2,:), pList(3,:), 'filled', 'red');

axis equal
xlabel('X[m]')
ylabel('Y[m]')
zlabel('Z[m]')
view(-76.12, 56.23);

%trisurf(k, pList(1,:)', pList(2,:)', pList(3,:)','FaceColor','red','FaceAlpha',0.1)

legend({'Ear Cavity', 'Ossicles', 'Reachable points'});
title(['Reachable points with ' num2str(n) ' cutouts']);

set(gca,'FontSize',18);