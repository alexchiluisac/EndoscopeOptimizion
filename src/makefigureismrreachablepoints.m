%% Create a figure showing the reachable points inside the ear (output of RRT)
%  for the ISMR 2020 paper
clear, close all

addpath('kinematics')
addpath('path-planning')
addpath('utils')

load ('results/atlas/round2/5-simulation.mat');
%figure('units','normalized','outerposition', [0 0 1 1])
figure
hold on

pathStl = fullfile('..', 'anatomical-models', modelID, 'me.stl');
[vertices, faces, ~, ~] = stlRead(pathStl);
earModel.vertices = vertices;
earModel.faces = faces;
stlPlot(earModel.vertices*1e3, earModel.faces, '');
stlPlot(osModel.vertices*1e3, osModel.faces, '', 10);

scatter3(pList(1,:)*1e3, pList(2,:)*1e3, pList(3,:)*1e3, 'filled', 'red');

axis equal
xlabel('X[mm]')
ylabel('Y[mm]')
zlabel('Z[mm]')
view(-118.5, 37.74);

%trisurf(k, pList(1,:)', pList(2,:)', pList(3,:)','FaceColor','red','FaceAlpha',0.1)

legend({'Ear Cavity', 'Ossicles', 'Reachable points'});
%title(['Reachable points with ' num2str(n) ' cutouts']);

set(gca,'FontSize',18);