% collisionTesting
% To try different collision algorithms

f = figure;

[meModel, transform, meModelRayCast, meSegmentation] = loadEarModel('175');
meModel.me
patch(meModel.me, 'FaceAlpha', 0.5, 'FaceColor', '#2b2e33', 'EdgeColor', '#0b1119');
patch(meModel.os, 'FaceAlpha', 0.5);
hold on;
% patch(app.osMesh, 'LineWidth', 0.2, 'FaceColor', '#071938');

configuration = [0.75 , 3.145 , 2];
cutouts.w = [1 1 1 1];
cutouts.u = [1 1 1 1];
cutouts.h = [1 1 1 1];
cutouts.alpha = [pi/2 0 0 0];

transform(1, 4) = transform(1, 4) + 10;
transform(2, 4) = transform(2, 4) + 20;

wrist = Wrist(1.65, 1.85, 4, cutouts);
wrist.fwkine(configuration, transform);

X = wrist.pose(1, :);
Y = wrist.pose(2, :);
Z = wrist.pose(3, :);

robotModel = wrist.makePhysicalModel();
meModel.me.vertices = meModel.me.Vertices;
meModel.os.vertices = meModel.os.Vertices;

X = robotModel.surface.X;
Y = robotModel.surface.Y;
Z = robotModel.surface.Z;

X = rmmissing(X);
Y = rmmissing(Y);
Z = rmmissing(Z);

matrixion = zeros(size(robotModel.surface));

totalX = X(:);
totalY = Y(:);
totalZ = Z(:);

total = [totalX totalY totalZ]
lol.vertices = total;

CollisionDetection(meModel.me, lol)
% This creates a convex hull, might be useful later on
% K = convhull(meModel.me.Vertices)
% trimesh(K, meModel.me.Vertices(:,1), meModel.me.Vertices(:, 2), meModel.me.Vertices(:, 3));
X = robotModel.surface.X;
Y = robotModel.surface.Y;
Z = robotModel.surface.Z;
surface(X, Y, Z, 'FaceColor', ...
    '#5cb5db', 'FaceLighting','gouraud', ...
    'AmbientStrength',0.5, 'EdgeColor', '#585d68', 'LineWidth', 0.003);

hold off;
axis auto;
axis equal;
campos('auto');
view(-135,35);
grid on;
