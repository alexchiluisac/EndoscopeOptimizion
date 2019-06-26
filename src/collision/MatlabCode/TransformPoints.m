function Points = TransformPoints(H, Points, ColumnPoints)
% Michael Siebold
% 3/15/2017
% if H is a 4x4 array it is treated as a homogeneous transform
% if H is a 3x3 array it is treated as a rotation matrix
% if H is a 3x1 array it is treated as a translation matrix
%
% points is a 3xmxk or nx3xk sized array of points
%
% ColumnPoints -> logical flag only takes over when Points is 3x3xk
%                 True (default)  -> points are stored as 3xmxk
%                 False           -> Points are stored as nx3xk
%
% Assumes Points are in R3
%
% up to 3 dimensional point storage
% outputs points in the same form it's given

if nargin<3, ColumnPoints = true;end
[m,n,k] = size(Points);
nPoints = numel(Points)/3;
PointsInRows = false; 

if m == 3 && ~ColumnPoints
    Points = permute(Points, [2,1,3]);
    PointsInRows = true;
end

if k == 1
    if numel(H) == 16
        Points = H*[Points; zeros(1, nPoints)];
        Points = Points(1:3,:);
    elseif numel(H) == 9
        Points = H*Points;
    elseif numel(H) == 3
        Points = Points + repmat(H, 1, nPoints);
    else
        error('Invalid transformation (Homogeneous (4x4), Rotation (3x3), and Translation (3x1) are supported.')
    end
else
    if numel(H) == 16
        Points = H*[reshape(Points, 3, nPoints, 1); ones(1, nPoints)];
    elseif numel(H) == 9
        Points = H*reshape(Points, 3, nPoints, 1);
    elseif numel(H) == 3
        Points = reshape(Points, 3, nPoints, 1) + repmat(H, 1, nPoints);
    else
        error('Invalid transformation (Homogeneous (4x4), Rotation (3x3), and Translation (3x1) are supported.')
    end
    Points = reshape(Points(1:3,:), [m, n, k]);
end

if PointsInRows
    Points = permute(Points, [2,1,3]);
end
