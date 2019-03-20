function h = stlPlot(v, f, name, seenMap)
%STLPLOT is an easy way to plot an STL object
%V is the Nx3 array of vertices
%F is the Mx3 array of faces
%NAME is the name of the object, that will be displayed as a title

figure, hold on
object.vertices = v;
object.faces = f;

if nargin < 4
    h = patch(object,'FaceColor', [0.8 0.8 1.0], ...
        'EdgeColor',       'none',        ...
        'FaceLighting',    'gouraud',     ...
        'FaceAlpha', 0.4, ...
        'AmbientStrength', 0.15);
else
    colorMap = ones(length(v), 1);
    colorMap(logical(seenMap)) = 5;
    h = patch(object,'FaceVertexCData', colorMap, ...
        'FaceColor', 'flat', ...
        'EdgeColor',       'none',        ...
        'FaceLighting',    'gouraud',     ...
        'FaceAlpha', 0.4, ...
        'AmbientStrength', 0.15);    
end

xlabel('X [mm]'), ylabel('Y [mm]'), zlabel('Z [mm]');

% Add a camera light, and tone down the specular highlighting
camlight('headlight');
material('dull');

% Fix the axes scaling, and set a nice view angle
axis('image');
view([-135 35]);
grid on;
title(name);
