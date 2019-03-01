function H = teach(robot, configuration)
% TEACH moves a robot to a given configuration and plots the corresponding
% pose.
%
%
% Authors: A. Chiluisa <ajchiluisa@wpi.edu>
%          L. Fichera  <lfichera@wpi.edu>
%
% Last revision: 3/1/2019

  [P, T] = robot.fwkine(configuration);

  X = P(1,:);
  Y = P(2,:);
  Z = P(3,:);
  
  scatter3(X, Y, Z, 100, 'r', 'filled');
  hold on, axis equal
  
  plot3(X, Y, Z, 'k', 'LineWidth', 2.5);
  xlabel('X[m]')
  ylabel('Y[m]')
  zlabel('Z[m]')
  
  triad('Matrix', eye(4), 'linewidth', 2.5);
  triad('Matrix', T(:,:,end), 'linewidth', 2.5);
end
