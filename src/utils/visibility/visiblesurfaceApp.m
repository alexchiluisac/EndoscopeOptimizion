function [seenFaces, seenVertices] = visiblesurfaceApp(app)

app.meMesh.vertices = app.meMesh.Vertices';
app.meMesh.faces = app.meMesh.Faces';

% Location behind the head
xi = app.wrist.pose(1, end - 1);
yi = app.wrist.pose(2, end - 1);
zi = app.wrist.pose(3, end - 1);

% Location of the head
xf = app.wrist.pose(1,end);
yf = app.wrist.pose(2,end);
zf = app.wrist.pose(3,end);

% Difference between the two
diffx = xf - xi;
diffy = yf - yi;
diffz = zf - zi;

magVec = sqrt((diffx)^2 + (diffy)^2 + (diffz)^2);
unitVec = [diffx/magVec, diffy/magVec, diffz/magVec];
notUnitVec = [diffx, diffy, diffz];

[seenFaces, seenVertices] = visibilitymap([xf yf zf], notUnitVec, app.meMesh);
end
