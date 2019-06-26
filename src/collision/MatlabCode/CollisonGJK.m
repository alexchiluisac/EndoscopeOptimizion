function [Collision, Simplex, dvec] = CollisonGJK(S1, S2, ResultsPlotting)
% Michael Siebold 2/1/2017 based on tutorial found at
% http://www.dyn4j.org/2010/04/gjk-gilbert-johnson-keerthi/
% rewritten after watching the tutorial posted at 
% https://mollyrocket.com/849
% 
% S1 is a 3xn1 set of points whose convex hull defines a shape to be tested
% S2 is a 3xn2 set of points whose convex hull defines a shape to be tested
% 
% ResultsPlotting -> true / false (default) flag to enable plotting
% 
DetailPlotting = false;
if nargin < 3, ResultsPlotting = false;end

dvec = [0;1;0];
Simplex = S1(:,1) - S2(:,1);

if DetailPlotting || ResultsPlotting
    global MinkDiff %#ok<*TLEV>
    MinkDiff = MinkowskiDifference(S1, S2);
end
ct = 0;
LoopLimit = 200;
while true && ct < LoopLimit
    % add a new point to the simplex because we haven't terminated yet
    Simplex = [Simplex, Support(S1, S2, dvec)]; %#ok<*AGROW>
    % make sure the last point we added actually passed the origin
    if Simplex(:,end).'*dvec <= 0
        %if the point added last was not past the origin in the direction
        %of d then the minkowski sum cannot possibly contain the origin
        %since the last point added is on the edge of the Minkoski
        %Difference
        Collision = false;
        PlotResults
        return
    else
        %otherwise we need to determine if the origin is in the current
        %simplex
        [Collision, Simplex, dvec] = ContainsOrigin(Simplex, dvec, DetailPlotting);
        if Collision
            PlotResults
            return
        end
    end
    ct = ct+1;
end

    function PlotResults
        if ResultsPlotting
            if Collision
                Colors = {'r', 'r'};
            else
                Colors = {'b', 'g'};
            end

            if size(S1, 2) > 1
                if size(S1, 2) == 8
                    PlotOABB(S1, 'facealpha', 0, 'edgecolor', Colors{1})
                else
                    trisurf(convhull(S1.'), S1(1,:), S1(2,:), S1(3,:), 'facealpha', '0.1', 'edgealpha', '0', 'facecolor', Colors{1})
                end
            else
                PlotPoints(S1, 'marker', '*', 'color', Colors{1})
            end
            if size(S2, 2) > 1
                if size(S2, 2) == 8
                    PlotOABB(S2, 'facealpha', 0, 'edgecolor', Colors{2})
                else
                    trisurf(convhull(S2.'), S2(1,:), S2(2,:), S2(3,:), 'facealpha', '0.1', 'edgealpha', '0', 'facecolor', Colors{2})
                end
            else
                PlotPoints(S2, 'marker', '*', 'color', Colors{2})
            end
            keyboard
        end
    end
PlotResults
end
function PlotOABB(S, varargin)

faces = [1,2,4,3;1,5,6,2;5,6,8,7;7,3,4,8;2,6,8,4;1,5,7,3];

if isempty(varargin)
    patch('vertices', S.', 'faces', faces, 'facealpha', 0)
else
    patch('vertices', S.', 'faces', faces, varargin{:})
end

end
        
function [CO, Simplex, dvec] = ContainsOrigin(Simplex, dvec, Plotting)

% Initialize "Contains Origin" to false since only the Tetrahedron case can
% contain the origin.
CO = false;

switch size(Simplex, 2)
    case 2
        [Simplex, dvec] = LineSegmentCase(Simplex, Plotting);
    case 3 
        [Simplex, dvec] = TriangleCase(Simplex, Plotting);
    case 4
        [Simplex, dvec, CO] = TetrahedronCase(Simplex, Plotting);
end
end

function [Simplex, dvec] = LineSegmentCase(Simplex, Plotting)
if size(Simplex, 2) ~= 2
    error('Incorrect Simplex for linear case.')
end
ab = Simplex(:,1) - Simplex(:,2);
ao = -Simplex(:,2);
if Plotting
    global MinkDiff
    figureha
    PlotPoints(MinkDiff)
    PlotPoints([0;0;0], 'linestyle', 'none', 'marker', '*', 'color', 'k')
    PlotPoints(Simplex, 'linestyle', '-', 'marker', 'none', 'color', 'k') % line segment ab
    PlotPoints(Simplex(:,2), 'linestyle', 'none', 'marker', '*', 'color', 'r') % point a
    PlotPoints(Simplex(:,1), 'linestyle', 'none', 'marker', '*', 'color', 'g') % point b
end
if PosDotProduct(ab, ao)
    %In region 1
    % if the origin is between the parallel planes orthogonal to
    % the line segment ab containing points a and b, set the
    % direction vector to point toward the origin perpendicular to
    % the line segment ab.
    dvec = TripleProduct(ab, ao, ab);
else
    % In region 2.
    % else remove point b and choose ao as the direction vector
    dvec = ao;
    Simplex = Simplex(:,2);
end
if Plotting
    PlotPoints([mean(Simplex,2), 0.5*norm(ao)*dvec/norm(dvec) + mean(Simplex,2)], 'linestyle', '-', 'marker', '*', 'color', 'g')
end
end

function [Simplex, dvec] = TriangleCase(Simplex, Plotting)
if size(Simplex, 2) ~= 3
    error('Incorrect Simplex for triangle case.')
end
Edge1 = Simplex(:,2) - Simplex(:,3);
Edge2 = Simplex(:,1) - Simplex(:,3);
ao = -Simplex(:,3);

if Plotting
    figureha
    global MinkDiff
    PlotPoints(MinkDiff)
    PlotPoints([0;0;0], 'linestyle', 'none', 'marker', '*', 'color', 'k')
    abh = PlotPoints(Simplex(:, [3,2]), 'linestyle', '-', 'marker', 'none', 'color', 'r'); % line segment ab
    ach = PlotPoints(Simplex(:, [3,1]), 'linestyle', '-', 'marker', 'none', 'color', 'g'); % line segment ac
    ah = PlotPoints(Simplex(:,3), 'linestyle', 'none', 'marker', '*', 'color', 'r'); % point a
    bh = PlotPoints(Simplex(:,2), 'linestyle', 'none', 'marker', '*', 'color', 'g'); % point b
    ch = PlotPoints(Simplex(:,1), 'linestyle', 'none', 'marker', '*', 'color', 'b'); % point c
    trih = patch('faces', [1,2,3], 'vertices', Simplex.', 'facealpha', 1, 'facecolor', 'r', 'edgealpha', 0);
end


if PosDotProduct(TripleProduct(Edge1, Edge2, Edge2), ao)
    if PosDotProduct(Edge2, ao)
        % in region 1
        Simplex = Simplex(:, [1, 3]);
        if Plotting
            delete([abh, bh])
        end
        dvec = TripleProduct(Edge2, ao, Edge2);
    else
        Star
    end
else
    if PosDotProduct(TripleProduct(Edge1, Edge1, Edge2), ao)
        Star
    else
        abc = hat(Edge1)*Edge2;
        if PosDotProduct(abc, ao)
            dvec = abc;
        else
            Simplex = Simplex(:, [2,1,3]);
            if Plotting
                delete([abh, ach, bh, ch])
                abh = PlotPoints(Simplex(:, [3,2]), 'linestyle', '-', 'marker', 'none', 'color', 'r'); % line segment ab
                ach = PlotPoints(Simplex(:, [3,1]), 'linestyle', '-', 'marker', 'none', 'color', 'g'); % line segment ac
                bh = PlotPoints(Simplex(:,2), 'linestyle', 'none', 'marker', '*', 'color', 'g'); % point b
                ch = PlotPoints(Simplex(:,1), 'linestyle', 'none', 'marker', '*', 'color', 'b'); % point c
            end
            dvec = -abc;
        end
    end
end

if Plotting
    PlotPoints([mean(Simplex,2), 0.5*norm(mean(Simplex,2))*dvec/norm(dvec) + mean(Simplex,2)], 'linestyle', '-', 'marker', '*', 'color', 'g')
end

    function Star
        if PosDotProduct(Edge1, ao)
            % in region 4
            Simplex = Simplex(:,[2,3]);
            dvec = TripleProduct(Edge1,ao,Edge1);
        else
            %in region 5
            Simplex = Simplex(:,end);
            dvec = ao;
            if Plotting
                delete(abh, ach, bh, ch, trih)
            end
        end
    end
end

function [Simplex, dvec, CO] = TetrahedronCase(Simplex, Plotting)
if size(Simplex, 2) ~= 4
    error('Incorrect Simplex for tetrahedron case.')
end
if Plotting
    figureha
    global MinkDiff
    PlotPoints(MinkDiff)
    PlotPoints([0;0;0], 'linestyle', 'none', 'marker', '*', 'color', 'k')
    PlotPoints(Simplex(:,4), 'linestyle', 'none', 'marker', '*', 'color', 'r'); % point a
    PlotPoints(Simplex(:,3), 'linestyle', 'none', 'marker', '*', 'color', 'g'); % point b
    PlotPoints(Simplex(:,2), 'linestyle', 'none', 'marker', '*', 'color', 'b'); % point c
    PlotPoints(Simplex(:,1), 'linestyle', 'none', 'marker', '*', 'color', 'k'); % point c
    
    PlotPoints(Simplex(:, [4,3]), 'linestyle', '-', 'marker', 'none', 'color', 'r'); % line segment ab
    PlotPoints(Simplex(:, [4,2]), 'linestyle', '-', 'marker', 'none', 'color', 'g'); % line segment ac
    PlotPoints(Simplex(:, [4,1]), 'linestyle', '-', 'marker', 'none', 'color', 'b'); % line segment ac
    
    patch('faces', [4, 3, 2], 'vertices', Simplex.', 'facealpha', 1, 'facecolor', 'r', 'edgealpha', 0); % triangle abc
    patch('faces', [4, 2, 1], 'vertices', Simplex.', 'facealpha', 1, 'facecolor', 'g', 'edgealpha', 0); % triangle abc
    patch('faces', [4, 3, 1], 'vertices', Simplex.', 'facealpha', 1, 'facecolor', 'b', 'edgealpha', 0); % triangle abc
end

CO = false; % initialize contains origin
% compute the edges
ab = Simplex(:, 3)-Simplex(:,4);
ac = Simplex(:, 2)-Simplex(:,4);
ad = Simplex(:, 1)-Simplex(:,4);
ao = -Simplex(:,4);

% a = Simplex(:, 4)
% b = Simplex(:, 3)
% c = Simplex(:, 2)
% d = Simplex(:, 1)
% All face normals are calculated such that they point away from the
% tetrahedron.
% F1 => the face normal of triange abc
% F2 => the face normal of triange acd
% F3 => the face normal of triange abd

if PosDotProduct(hat(ab)*ac, ao) % compute face normal with a cross product, then compare to ao
    % if the origin is opposite F1
    if PosDotProduct(hat(ac)*ad, ao) 
        % If the origin is also opposite F2, remove the points opposite the
        % edge of the tetrahedron on both F1 and F2 (points b and d).
        Simplex(:, [3,1]) = [];
        % Call linear routine
        [Simplex, dvec] = LineSegmentCase(Simplex, Plotting);
    elseif PosDotProduct(hat(ad)*ab, ao)
        % If the origin is also opposite F3, remove the points opposite the 
        % edge of the tetrahedron on both F1 and F3 (points c and d).
        Simplex(:, 1:2) = [];
        % Call linear routine
        [Simplex, dvec] = LineSegmentCase(Simplex, Plotting);
    else
        % If the origin is only opposite F1, remove the point opposite to
        % F1 (point d).
        Simplex(:, 1) = [];
        % Call triangle routine
        [Simplex, dvec] = TriangleCase(Simplex, Plotting);
    end
elseif PosDotProduct(hat(ac)*ad, ao)
    % If the origin is opposite F2 and not opposite F1
    if PosDotProduct(hat(ad)*ab, ao)
        % If the origin is also opposite F3, remove the points opposite the
        % edge of the tetrahedron on both F2 and F3 (points c and b).
        Simplex(:,2:3) = [];
        % Call linear routine
        [Simplex, dvec] = LineSegmentCase(Simplex, Plotting);
    else
        % If the origin is only opposite F2, remove the point opposite to
        % F2 (point b).
        Simplex(:, 3) = [];
        % Call triangle case
        [Simplex, dvec] = TriangleCase(Simplex, Plotting);
    end
elseif PosDotProduct(hat(ad)*ab, ao)
    % if the origin is opposite F3 only, then remove the point opposite F3
    % (point c)
    Simplex(:, 2) = [];
    % Call the triangle routine
    [Simplex, dvec] = TriangleCase(Simplex, Plotting);
else
    dvec = [];
    CO = true;
end
end

function SD = PosDotProduct(v1, v2)
    SD = v1.'*v2 > 0;
end
