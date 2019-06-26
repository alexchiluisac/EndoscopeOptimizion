function Collision = CollisionSAT(ACorners, C1, BCorners, C2, Plotting)
if nargin < 5, Plotting = false;end

[A, a] = FindBoxAxesAndExtents(ACorners);
[B, b] = FindBoxAxesAndExtents(BCorners);
D = (C2(:) - C1(:)).';

L = [A, B,... 
     hat(A(:,1))*B(:,1), hat(A(:,1))*B(:,2), hat(A(:,1))*B(:,3),...
     hat(A(:,2))*B(:,1), hat(A(:,2))*B(:,2), hat(A(:,2))*B(:,3),...
     hat(A(:,3))*B(:,1), hat(A(:,3))*B(:,2), hat(A(:,3))*B(:,3)];

R = abs(sum(L.*repmat(D(:), 1, 15), 1)).';
R1 = CalculateRi(L, A, a);
R2 = CalculateRi(L, B, b);

Collision = all(R <= R1+R2);
if Plotting
    CollisionPlotting(Collision, ACorners, BCorners)
end
end

function [A, a] = FindBoxAxesAndExtents(BoxCorners)
A(:,1) = BoxCorners(:,5) - BoxCorners(:,1);
A(:,2) = BoxCorners(:,3) - BoxCorners(:,1);
A(:,3) = BoxCorners(:,2) - BoxCorners(:,1);

a = [norm(A(:,1)); norm(A(:,2)); norm(A(:,3))];
A = A./repmat(a.', 3, 1);
a = a/2;
end

function R = CalculateRi(L, Arr, arr)
LdA1 = sum(L.*repmat(Arr(:,1), 1, 15), 1);
LdA2 = sum(L.*repmat(Arr(:,2), 1, 15), 1);
LdA3 = sum(L.*repmat(Arr(:,3), 1, 15), 1);
R = (arr(1)*sign(LdA1).*LdA1 + arr(2)*sign(LdA2).*LdA2 + arr(3)*sign(LdA3).*LdA3).';
end

function CollisionPlotting(Collision, ACorners, BCorners)
if Collision
    PlotOABB(ACorners, 'color', 'm')
    PlotOABB(BCorners, 'color', 'm')
else
    PlotOABB(ACorners, 'color', 'c')
    PlotOABB(BCorners, 'color', 'k')
end
end

function varargout = PlotOABB(Corners, varargin)

hg = hggroup;

LineChars = {'parent', hg};
PointChars = {'linestyle', 'none', 'marker', '*', 'parent', hg};

PlotPoints(Corners, varargin{:}, PointChars{:})
line(Corners(1,1:2), Corners(2,1:2), Corners(3,1:2), varargin{:}, LineChars{:})
line(Corners(1,3:4), Corners(2,3:4), Corners(3,3:4), varargin{:}, LineChars{:})
line(Corners(1,5:6), Corners(2,5:6), Corners(3,5:6), varargin{:}, LineChars{:})
line(Corners(1,7:8), Corners(2,7:8), Corners(3,7:8), varargin{:}, LineChars{:})

line(Corners(1,[1,3]), Corners(2,[1,3]), Corners(3,[1,3]), varargin{:}, LineChars{:})
line(Corners(1,[1,3,7,5,1]), Corners(2,[1,3,7,5,1]), Corners(3,[1,3,7,5,1]), varargin{:}, LineChars{:})
line(Corners(1,[2,4,8,6,2]), Corners(2,[2,4,8,6,2]), Corners(3,[2,4,8,6,2]), varargin{:}, LineChars{:})

if nargout>0
    varargout{1} = hg;
end
end
