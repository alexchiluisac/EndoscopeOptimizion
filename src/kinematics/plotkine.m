function H = plotkine(robot, configuration)
[P, ~] = robot.fwkine(configuration);

x = P(1,:);
y = P(2,:);
z = P(3,:);
scatter3(x,y,z,'r');
hold on, axis equal
plot3(x,y,z,'k');
xlabel('X[m]')
ylabel('Y[m]')
zlabel('Z[m]')
end
