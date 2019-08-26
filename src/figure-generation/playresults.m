%close all
clear, clc

addpath('kinematics')
addpath('path-planning')
addpath('utils')

load 10-simulation.mat

path = fullfile('..', 'anatomical-models', modelID);
pathStl = fullfile(path, 'me.stl');
[vertices, faces, ~, ~] = stlRead(pathStl);
earModel.vertices = vertices;
earModel.faces = faces;


figure('units','normalized','outerposition',[0 0 1 1])

% Visualize the robot inside the cavity
ii = 1;
h1 = stlPlot(earModel.vertices, earModel.faces, 'Collision detection test.');
stlPlot(osModel.vertices, osModel.faces, 'Collision detection test.');
hold on

ax = gca;
outerpos = ax.OuterPosition;
ti = ax.TightInset;
left = outerpos(1) + ti(1);
bottom = outerpos(2) + ti(2);
ax_width = outerpos(3) - ti(1) - ti(3);
ax_height = outerpos(4) - ti(2) - ti(4);
ax.Position = [left bottom ax_width ax_height];

robot.fwkine(qList(:,ii), T);
robotPhysicalModel = robot.makePhysicalModel();
h2 = surf(robotPhysicalModel.surface.X, ...
    robotPhysicalModel.surface.Y, ...
    robotPhysicalModel.surface.Z, ...
    'FaceColor','blue');
set(gca,'FontSize',14);

axis equal

%for ii = 1 : size(pList, 2)
while true
    robot.fwkine(qList(:,ii), T);
    robotPhysicalModel = robot.makePhysicalModel();
    
    h2.XData = robotPhysicalModel.surface.X;
    h2.YData = robotPhysicalModel.surface.Y;
    h2.ZData = robotPhysicalModel.surface.Z;
    title(['Pose ' num2str(ii) ' of ' num2str(size(pList, 2))]);
    
    
    %scatter3(pList(1,ii), pList(2,ii), pList(3,ii), 'filled', 'red');
    drawnow
    
%     fprintf('Press "n" to move forward or "p" to move back.\n')
%     fprintf('Press any other key to stop testing and generate the reachable workspace.\n\n')
%     
%     while ~waitforbuttonpress, end
%     k = get(gcf, 'CurrentCharacter');
    
%     switch k
%         case 'p'
%             ii = ii - 1;
%             if ii < 1, ii = 1; end
%         case 'n'
             ii = ii + 1;
             if ii > size(pList, 2), break; end
%         otherwise
%             break
%     end
end
