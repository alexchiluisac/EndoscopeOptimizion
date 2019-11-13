%% This script generates a curve based on desired curvature and torsion profiles
clc, clear, close all

col = distinguishable_colors(10);

% Define arc length, curvature and torsion
arcLength = 1e-3;
k = @(s) 5/s;
tau = @(s) 5e6*s;

%% *** NO NEED TO CHANGE THE CODE BELOW ***
% Numerically solve the Frenet-Serret equations
% t -> x(1) x(2) x(3)
% n -> x(4) x(5) x(6)
% b -> x(7) x(8) x(9)        
f = @(s,x) [k(s)*x(4);  k(s)*x(5); k(s)*x(6);
           -k(s)*x(1) + tau(s)*x(7); -k(s)*x(2) + tau(s)*x(8); -k(s)*x(3) + tau(s)*x(9);
           -tau(s)*x(4); -tau(s)*x(5); -tau(s)*x(6)];
       
[s,y] = ode45(f, [0 arcLength], [0 0 1 1 0 0 0 1 0]);

t = [y(:,1) y(:,2) y(:,3)]';
n = [y(:,4) y(:,5) y(:,6)]';
b = [y(:,7) y(:,8) y(:,9)]';

% Generate the arc points by integration of the t vector along s
arc = zeros(3, size(s, 1));

for ii = 2 : size(s, 1)
  arc(:,ii) = [trapz(s(1:ii), t(1,1:ii));
               trapz(s(1:ii), t(2,1:ii));
               trapz(s(1:ii), t(3,1:ii))];
end

% Plot the resulting line
figure
scatter3(arc(1,:), arc(2,:), arc(3,:),'MarkerEdgeColor', col(1,:), 'LineWidth', 2.5);   
hold on, axis equal, grid on
xlim([-1e-3 1e-3]), ylim([-1e-3 1e-3]), zlim([0 1.5e-3]);
xlabel('X [m]'), ylabel('Y [m]'), zlabel('Z [m]'), 
view(80, 25);
set(gca,'FontSize',16);

h = triad('scale', 1e-3/2, 'linewidth', 2.5);

% Make an animation showing the Frenet-Serret frames
for ii = 2 : size(s, 1)
    rot = [n(:,ii) b(:,ii) t(:,ii)];
    transl = arc(:,ii);
    T = [rot transl; 0 0 0 1];
    
    h.Matrix = T;
    pause(0.1);
    drawnow
end