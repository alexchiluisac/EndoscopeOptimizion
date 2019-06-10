function varargout = MinDistGJK(S1, S2, Simplex, dvec, ResultsPlotting)
% Michael Siebold 2/1/2017 based on tutorial found at
% http://www.dyn4j.org/2010/04/gjk-distance-closest-points/
% 
% S1 is a 3xn1 set of points whose convex hull defines a shape to be tested
% S2 is a 3xn2 set of points whose convex hull defines a shape to be tested
% 
% If MinDistGJK is being implemented after CollisionGJK has indicated that
% no collision has occured, it may improve speed (slightly) to initialize
% MinDiskGJK's 'Simplex' and 'dvec' parameters. These inputs will not be 
% commonly used.
% 
% Simplex
% dvec
% 
% ResultsPlotting -> true / false (default) flag to enable plotting
% [MD, cpS1, cpS2] = GJKMinDist3D(S1, S2)
% 

tol = 1e-10;
DetailPlotting = false;
if nargin < 5, ResultsPlotting = false;end

if nargin < 3 || isempty(Simplex)
    % Points from S1 and S2 used to generate the simplex
    S1Points = S1(:,1); 
    S2Points = S2(:,1); 
    Simplex = S1Points - S2Points;
    dvec = -Simplex; % [0;1;0];
end

if DetailPlotting || ResultsPlotting
    global MinkDiff %#ok<*TLEV>
    MinkDiff = MinkowskiDifference(S1, S2);
end
ct = 0;
LoopLimit = 200;
while true && ct < LoopLimit
    if isnan(Simplex)
        varargout = assignOutputs(nan, [], [], nargout);
        PlotResults
        return
    end
    if all(abs(dvec) < tol)
        % implies that the objects are touching and therefore in collision
        varargout = assignOutputs(0, [], [], nargout);
        PlotResults
        return
    end
    
    % Find a new point to add to the simplex
    [SimplexNP, S1PointsNP, S2PointsNP] = Support(S1, S2, dvec);
    
    if any(all(Simplex == repmat(SimplexNP, 1, size(Simplex, 2)), 1))
        % If the new point does not drive the simplex closer to the origin,
        % then terminate the algorithm.
        if nargout > 1
            % If the closest points are required, then compute them.
            [MD, cpS1, cpS2] = DetermineClosestPoints(Simplex, S1Points, S2Points);
        else
            cpS1 = []; cpS2 = [];
        end
        varargout = assignOutputs(MD, cpS1, cpS2, nargout);
        PlotResults
        return
    end
    
    Simplex = [Simplex, SimplexNP]; %#ok<*AGROW>
    S1Points = [S1Points, S1PointsNP];
    S2Points = [S2Points, S2PointsNP];

    [Simplex, dvec, S1Points, S2Points] = ClosestToOrigin(Simplex, dvec, DetailPlotting, S1Points, S2Points);
    ct = ct+1;
end

    function PlotResults
        if ResultsPlotting
            figure
            hold on
            axis equal
            set(gcf, 'color', [1,1,1])

            Colors = {'b', 'g'};
            PlotPoints(S1, [], 'linestyle', 'none','marker', '*', 'color', Colors{1})
            PlotPoints(S2, [], 'linestyle', 'none','marker', '*', 'color', Colors{2})
            if size(S1, 2) > 1
                trisurf(convhull(S1.'), S1(1,:), S1(2,:), S1(3,:), 'facealpha', '0.3', 'edgealpha', '0', 'facecolor', Colors{1})
            end
            if size(S2, 2) > 1
                trisurf(convhull(S2.'), S2(1,:), S2(2,:), S2(3,:), 'facealpha', '0.3', 'edgealpha', '0', 'facecolor', Colors{2})
            end
            PlotPoints([cpS1, cpS2], [], 'linestyle', '-', 'color', 'r')
        end
    end

if ct >= LoopLimit 
    warning('Max loop iteration reached.')
end
end



function [Simplex, dvec, S1Points, S2Points] = ClosestToOrigin(Simplex, dvec, Plotting, S1Points, S2Points)


switch size(Simplex, 2)
    case 2
        [Simplex, dvec, S1Points, S2Points] = LineSegmentCase(Simplex, Plotting, S1Points, S2Points);
    case 3 
        [Simplex, dvec, S1Points, S2Points] = TriangleCase(Simplex, Plotting, S1Points, S2Points);
    case 4
        [Simplex, dvec, S1Points, S2Points] = TetrahedronCase(Simplex, Plotting, S1Points, S2Points);
end
end


function [Simplex, dvec, S1Points, S2Points] = LineSegmentCase(Simplex, Plotting, S1Points, S2Points)
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
    
    [Simplex, S1Points, S2Points] = ReduceSimplex(2, Simplex, S1Points, S2Points);
end
if Plotting
    PlotPoints([mean(Simplex,2), 0.5*norm(ao)*dvec/norm(dvec) + mean(Simplex,2)], 'linestyle', '-', 'marker', '*', 'color', 'g')
end
end

function [Simplex, dvec, S1Points, S2Points] = TriangleCase(Simplex, Plotting, S1Points, S2Points)
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
    PlotPoints(Simplex(:,3), 'linestyle', 'none', 'marker', '*', 'color', 'r'); % point a
    bh = PlotPoints(Simplex(:,2), 'linestyle', 'none', 'marker', '*', 'color', 'g'); % point b
    ch = PlotPoints(Simplex(:,1), 'linestyle', 'none', 'marker', '*', 'color', 'b'); % point c
    trih = patch('faces', [1,2,3], 'vertices', Simplex.', 'facealpha', 1, 'facecolor', 'r', 'edgealpha', 0);
end


if PosDotProduct(TripleProduct(Edge1, Edge2, Edge2), ao)
    if PosDotProduct(Edge2, ao)
        % in region 1
        [Simplex, S1Points, S2Points] = ReduceSimplex([1, 3], Simplex, S1Points, S2Points);
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
            [Simplex, S1Points, S2Points] = ReduceSimplex([2,1,3], Simplex, S1Points, S2Points);
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
            [Simplex, S1Points, S2Points] = ReduceSimplex([2, 3], Simplex, S1Points, S2Points);
            dvec = TripleProduct(Edge1,ao,Edge1);
        else
            %in region 5
            [Simplex, S1Points, S2Points] = ReduceSimplex(size(Simplex,2), Simplex, S1Points, S2Points);
            dvec = ao;
            if Plotting %&& exist('abh', 'var')
                delete([abh, ach, bh, ch, trih])
            end
        end
    end
end

function [Simplex, dvec, S1Points, S2Points] = TetrahedronCase(Simplex, Plotting, S1Points, S2Points)
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

% a = Simplex(:, 4)
% b = Simplex(:, 3)
% c = Simplex(:, 2)
% d = Simplex(:, 1)
% All face normals are calculated such that they point away from the
% tetrahedron.
% F1 => the face normal of triange abc %% columns 2:4
% F2 => the face normal of triange acd %% columns [1,2,4]
% F3 => the face normal of triange abd %% columns [1,3,4]

% compute the edges
ab = Simplex(:, 3)-Simplex(:,4);
ac = Simplex(:, 2)-Simplex(:,4);
ad = Simplex(:, 1)-Simplex(:,4);
ao = -Simplex(:,4);

if PosDotProduct(hat(ab)*ac, ao) % compute face normal with a cross product, then compare to ao
    % if the origin is opposite F1
    if PosDotProduct(hat(ac)*ad, ao) 
        if pointTriangleDistance(Simplex(:,2:4).',[0,0,0]) < pointTriangleDistance(Simplex(:,[1,2,4]).',[0,0,0]) 
            % If F1 is closer to the origin than F2, remove the point
            % opposite F1 (point d). 
            [Simplex, S1Points, S2Points] = ReduceSimplex(2:4, Simplex, S1Points, S2Points);
        else
            % If F2 is closer to the origin than F1, remove the point
            % opposite F2 (point b). 
            [Simplex, S1Points, S2Points] = ReduceSimplex([1,2,4], Simplex, S1Points, S2Points);
        end
        [Simplex, dvec, S1Points, S2Points] = TriangleCase(Simplex, Plotting, S1Points, S2Points);
    elseif PosDotProduct(hat(ad)*ab, ao)
        if pointTriangleDistance(Simplex(:,2:4).',[0,0,0]) < pointTriangleDistance(Simplex(:,[1,3,4]).',[0,0,0]) 
            % If F1 is closer to the origin than F3, remove the point
            % opposite F1 (point d). 
            [Simplex, S1Points, S2Points] = ReduceSimplex(2:4, Simplex, S1Points, S2Points);
        else
            % If F3 is closer to the origin than F1, remove the point
            % opposite F3 (point c). 
            [Simplex, S1Points, S2Points] = ReduceSimplex([1,3,4], Simplex, S1Points, S2Points);
        end
        [Simplex, dvec, S1Points, S2Points] = TriangleCase(Simplex, Plotting, S1Points, S2Points);
    else
        % If the origin is only opposite F1, remove the point opposite to
        % F1 (point d).
        [Simplex, S1Points, S2Points] = ReduceSimplex(2:4, Simplex, S1Points, S2Points);
        % Call triangle routine
        [Simplex, dvec, S1Points, S2Points] = TriangleCase(Simplex, Plotting, S1Points, S2Points);
    end
elseif PosDotProduct(hat(ac)*ad, ao)
    % If the origin is opposite F2 and not opposite F1
    if PosDotProduct(hat(ad)*ab, ao)
        if pointTriangleDistance(Simplex(:,[1,2,4]).',[0,0,0]) < pointTriangleDistance(Simplex(:,[1,3,4]).',[0,0,0]) 
            % If F2 is closer to the origin than F3, remove the point
            % opposite F2 (point b). 
            [Simplex, S1Points, S2Points] = ReduceSimplex([1,2,4], Simplex, S1Points, S2Points);
        else
            % If F3 is closer to the origin than F2, remove the point
            % opposite F3 (point c). 
            [Simplex, S1Points, S2Points] = ReduceSimplex([1,3,4], Simplex, S1Points, S2Points);
        end
        [Simplex, dvec, S1Points, S2Points] = TriangleCase(Simplex, Plotting, S1Points, S2Points);
    else
        % If the origin is only opposite F2, remove the point opposite to
        % F2 (point b).
        [Simplex, S1Points, S2Points] = ReduceSimplex([1:2,4], Simplex, S1Points, S2Points);
        % Call triangle case
        [Simplex, dvec, S1Points, S2Points] = TriangleCase(Simplex, Plotting, S1Points, S2Points);
    end
elseif PosDotProduct(hat(ad)*ab, ao)
    % if the origin is opposite F3 only, then remove the point opposite F3
    % (point c)
    [Simplex, S1Points, S2Points] = ReduceSimplex([1,3:4], Simplex, S1Points, S2Points);
    % Call the triangle routine
    [Simplex, dvec, S1Points, S2Points] = TriangleCase(Simplex, Plotting, S1Points, S2Points);
else
    % The point is contained in the simplex, so the objects are in
    % collision.
    [Simplex, dvec, S1Points, S2Points] = deal(nan);
%     error('Objects in collision.')
end

end

function [cp, cpm] = Simplex2OriginClosesetPoint(Simplex)

switch size(Simplex, 2)
    case 1
        cp = Simplex;
        cpm = norm(Simplex);
    case 2
        % create the lines
        ab = Simplex(:,1) - Simplex(:,2);
        ao = -Simplex(:,2);
        % project ao onto ab
        aoProjOntoab = dot(ao, ab);
        % get the length squared
        L2 = dot(ab, ab);
        % calculate the distance along ab
        t = aoProjOntoab/L2;

        %     calculate the closest point
        %     if the closest point is not on the line segment, choose an endpoint
        if t <= 0 
            cp = Simplex(:,2);
        elseif t >= 1
            cp = Simplex(:,1);
        else
            cp = t*ab + Simplex(:,2);
        end
        cpm = norm(cp);
    case 3
        [cpm, cp(:)] = pointTriangleDistance(Simplex.',[0,0,0]);
end
end

function SD = PosDotProduct(v1, v2)
    SD = v1.'*v2 > 0;
end

function cellout = assignOutputs(distance, cpA, cpB, noutputs)
if noutputs > 0
    cellout{1} = distance;
end
if noutputs > 1
    cellout{2} = cpA;
end
if noutputs > 2
    cellout{3} = cpB;
end
end

function [MD, cpS1, cpS2] = DetermineClosestPoints(Simplex, S1Points, S2Points)
if size(Simplex, 2) == 1
    % if the simplex is only one point, then that point has to be closest
    cpS1 = S1Points;
    cpS2 = S2Points;
elseif size(Simplex, 2) == 2
    % if the simplex is a line, then find closest point on the line
    [cpS1, cpS2] = CalculateStructureClosestPointsLinear(Simplex, S1Points, S2Points);
elseif size(Simplex, 2) == 3
    % If the simplex is a triangle, then find the closest point on the
    % triangle.
    Lambda = Simplex\Simplex2OriginClosesetPoint(Simplex).';
%     Lambda = Simplex\cpM.';
    cpS1 = Lambda(1)*S1Points(:,1) + Lambda(2)*S1Points(:,2) + Lambda(3)*S1Points(:,3);
    cpS2 = Lambda(1)*S2Points(:,1) + Lambda(2)*S2Points(:,2) + Lambda(3)*S2Points(:,3);
end
    MD = sqrt(sum((cpS1 - cpS2).^2, 1));
end

function [cpA, cpB] = CalculateStructureClosestPointsLinear(Simplex, S1Points, S2Points)

    L = Simplex(:,2)-Simplex(:,1);
    if all(L==0)
        cpA = S1Points(:,1);
        cpB = S1Points(:,2);
        return
    end
    
    lambda2 = dot(-L, Simplex(:,1)) / dot(L, L);
    lambda1 = 1 - lambda2;
    
    if lambda1 < 0
        cpA = S1Points(:,2);
        cpB = S2Points(:,2);
        return
    elseif lambda2 < 0
        cpA = S1Points(:,1);
        cpB = S2Points(:,1);
        return
    end
    cpA = lambda1 * S1Points(:,1) + lambda2*S1Points(:,2);
    cpB = lambda1 * S2Points(:,1) + lambda2*S2Points(:,2);
end

function [Simplex, S1Points, S2Points] = ReduceSimplex(PreserveInds, Simplex, S1Points, S2Points)
Simplex = Simplex(:, PreserveInds);

if ~isempty(S1Points)
    S1Points = S1Points(:, PreserveInds); % S1Points
    S2Points = S2Points(:, PreserveInds); % S2Points
end
end