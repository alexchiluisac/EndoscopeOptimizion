function varargout = CollisionDetection(Object1, Object2, varargin)
% Michael Siebold
% 3/21/2017
% 
% Possible Inputs
%     Object1
%     Object2
% 
%   These are the two objects that will be tested for collision.
%   Valid input object types are: 
% 
%      Point  - A single point (3x1 vector)
%      OcTree - An octree of the class 'OcTree'
%      STL    - An STL with a collection of 'vertices'
%               that will be input into 'OcTree'
%      Object - A collection of points defining a
%               convex object. (3xn array)
%   The program will automatically decide which type of object it is given.
% 
% Possible Outputs
%     Collision   - Boolean value which is true when Object1 overlaps
%                   Object2
%     MinDist     - Minimum Distance between Object1 and Object2. 
%     CPObject1   - The point on Object1 that is closest to Object2.
%     CPObject2   - The point on Object1 that is closest to Object2.
%     
%     MinDist, CPObject1, and CPObject2 are only able to be calculated if 
%         the two Objects are not in collision.
% 
%  
% Example
%  This will likely yield two object not in collision.
%       S1 = rand(3,8);
%       S2 = rand(3,8) + repmat([1;2;3], 1, 8);
%       CollisionDetection(S1, S2, 'Ploting', true)
%  Creating and testing an additional object, S3, will likely result in a
%  collision 
%       S3 = rand(3,8) + repmat(0.5*[1;1;0], 1, 8);
%       CollisionDetection(S1, S3, 'Plotting', true)
% 
% 
% Additional Inputs (varagin style)
% 
%     Plotting    - true/false(default) plots the results of the collsion
%                   detection.
%     binCapacity - Maximum number of points a bin may contain. If more
%                   points exist, the bin will be recursively subdivided.
%                   Defaults to ceil(numPts/10).
%     maxDepth    - Maximum number of times a bin may be subdivided.
%                   Defaults to INF.
%     maxSize     - Maximum size of a bin edge. If any dimension of a bin 
%                   exceeds maxSize, it will be recursively subdivided.
%                   Defaults to INF.
%     minSize     - Minimum size of a bin edge. Subdivision will stop after 
%                   any dimension of a bin gets smaller than minSize.
%                   Defaults to 1000*eps.
%     style       - Either 'equal' (default) or 'weighted'. 'equal' 
%                   subdivision splits bins at their central coordinate
%                   (ie, one bin subdivides into 8 equally sized bins).
%                   'weighted' subdivision divides bins based on the mean
%                   of all points they contain. Weighted subdivision is
%                   slightly slower than equal subdivision for a large
%                   number of points, but it can produce a more efficient
%                   decomposition with fewer subdivisions.
%     Aligned     - 'PCA' (default) yields an octree with Object Aligned 
%                   Boundary Boxes (OABB). 'Axis' yields an octree with
%                   Axis Aligned Boundary Boxes (AABB). 
%     Voxelized   - Default 'false'. If this is enabled then in the deepest
%                   level of the octree a bin corresponds to a voxel. All
%                   boxes are of uniform dimension. Enabling this disables 
%                   option 'Aligned', 'PCA'. Enabling this also forces 
%                   'maxSize' = 1 (voxel).
IP = inputParser;
IP.addParameter('Plotting', false);
IP.addParameter('IntermediatePlotting', false);



if isfield(Object1, 'vertices')
    % Octree Parameters for Object 1 
    % (Only used if an 'OcTree' object is to be created for Obj1)
    IP.addParameter('Object1binCapacity', ceil(length(Object1.vertices))/10);
    IP.addParameter('Object1maxDepth', inf);
    IP.addParameter('Object1maxSize', inf);
    IP.addParameter('Object1minSize', 1000 * eps);
    IP.addParameter('Object1style', 'equal');
    IP.addParameter('Object1Aligned','PCA');
    IP.addParameter('Object1Voxelized', false);
end

if isfield(Object2, 'vertices')
    % Octree Parameters for Object 2 
    % (Only used if an 'OcTree' object is to be created for Obj2)
    IP.addParameter('Object2binCapacity', ceil(length(Object2.vertices))/10);
    IP.addParameter('Object2maxDepth', inf);
    IP.addParameter('Object2maxSize', inf);
    IP.addParameter('Object2minSize', 1000 * eps);
    IP.addParameter('Object2style', 'equal');
    IP.addParameter('Object2Aligned','PCA');
    IP.addParameter('Object2Voxelized', false);
end

IP.parse(varargin{:});
IPProperties = IP.Results;

if isfield(Object1, 'vertices')
    % If Obj1 is an STL convert it into an OcTree. 
    
    if size(Object1.vertices, 2) > 3
        Object1.vertices = Object1.vertices.';
    end
    Object1 = OcTree(Object1.vertices, 'Aligned',     IPProperties.Object1Aligned,...
                                       'maxSize',     IPProperties.Object1maxSize,...
                                       'minSize',     IPProperties.Object1minSize,...
                                       'Voxelized',   IPProperties.Object1Voxelized,...
                                       'binCapacity', IPProperties.Object1binCapacity,...
                                       'style',       IPProperties.Object1style,...
                                       'binCapacity', IPProperties.Object1binCapacity);
end

if isfield(Object2, 'vertices')
    % If Obj2 is an STL convert it into an OcTree. 
    if size(Object2.vertices, 2) > 3
        Object2.vertices = Object2.vertices.';
    end
    Object2 = OcTree(Object2.vertices, 'Aligned',     IPProperties.Object2Aligned,...
                                       'maxSize',     IPProperties.Object2maxSize,...
                                       'minSize',     IPProperties.Object2minSize,...
                                       'Voxelized',   IPProperties.Object2Voxelized,...
                                       'binCapacity', IPProperties.Object2binCapacity,...
                                       'style',       IPProperties.Object2style,...
                                       'binCapacity', IPProperties.Object2binCapacity);
end

if IPProperties.IntermediatePlotting
    figure
    hold on 
    axis equal
    set(gcf, 'color', [1,1,1])
end

if isa(Object1, 'OcTree') && isa(Object2, 'OcTree')
    % If both objects are octrees, call the Octree to Octree Collision
    % method. 
    Collision = CollisionOT2OT(0, Object1, Object2, IPProperties.IntermediatePlotting);
    
    if nargout > 1 && ~Collision
        [varargout{2}, varargout{3}, varargout{4}] = MinDistOT2OT(0, Object1, Object2, IPProperties.IntermediatePlotting);
    else
        [varargout{2}, varargout{3}, varargout{4}] = deal([]);
    end
    
elseif isa(Object1, 'OcTree')
    if size(Object2, 2) > 1
        Collision = CollisionObject2OT(0, Object1, Object2, [], true, IPProperties.IntermediatePlotting);
    else
        Collision = CollisionObject2OT(0, Object1, Object2, [], true, IPProperties.IntermediatePlotting);
    end
    
    if nargout > 1 && ~Collision 
        [varargout{2}, varargout{3}, varargout{4}] = MinDistObj2OT(0, Object1, Object2, IPProperties.IntermediatePlotting);
    else
        [varargout{2}, varargout{3}, varargout{4}] = deal([]);
    end
    
elseif isa(Object2, 'OcTree')
    if size(Object1, 2) > 1
        Collision = CollisionObject2OT(0, Object2, Object1, [], true, IPProperties.IntermediatePlotting);
    else
        Collision = CollisionObject2OT(0, Object2, Object1, [], true, IPProperties.IntermediatePlotting);
    end
    
    if nargout > 1 && ~Collision 
        [varargout{2}, varargout{3}, varargout{4}] = MinDistObj2OT(0, Object2, Object1, IPProperties.IntermediatePlotting);
    else
        [varargout{2}, varargout{3}, varargout{4}] = deal([], [], []);
    end
else
    % If both Object1 and Object2 are "Objects" or "Points"
    Collision = CollisonGJK(Object1, Object2, IPProperties.IntermediatePlotting);
    if nargout > 1 && ~Collision 
        [varargout{2}, varargout{3}, varargout{4}] = MinDistGJK(Object1, Object2, [], [], IPProperties.IntermediatePlotting);
    else
        [varargout{2}, varargout{3}, varargout{4}] = deal([]);
    end
end


varargout{1} = Collision;

if IPProperties.Plotting
    figure
    hold on
    axis equal
    set(gcf, 'color', [1,1,1])
    if Collision 
        Colors = {'r', 'r'};
    else
        Colors = {'g', 'b'};
    end
    
    if isa(Object1, 'OcTree')
        Object1.Plot(Object1.MaxBinDepth, [], 'edgecolor', Colors{1});
    else
        if size(Object1, 2) > 1
%             PlotPoints(Object1, 'marker', '*', 'color', Colors{1})
            trisurf(convhull(Object1.'), Object1(1,:), Object1(2,:), Object1(3,:),...
                'facealpha', '0.3', 'edgealpha', '0', 'facecolor', Colors{1})
        else
            PlotPoints(Object1, 'marker', 'x', 'color', Colors{1}, 'markersize', 30, 'linestyle', '-', 'linewidth', 10)
        end
    end
    
    if isa(Object2, 'OcTree')
        Object2.Plot(Object2.MaxBinDepth, [], 'edgecolor', Colors{1});
    else
        if size(Object2, 2) > 1
%             PlotPoints(Object2, 'marker', '*', 'color', Colors{2})
            trisurf(convhull(Object2.'), Object2(1,:), Object2(2,:), Object2(3,:),...
                'facealpha', '0.3', 'edgealpha', '0', 'facecolor', Colors{2})
        else
            PlotPoints(Object2, 'marker', 'x', 'color', Colors{2}, 'markersize', 30, 'linestyle', '-', 'linewidth', 10)
        end
    end
    
    if nargout > 2 && ~isempty(varargout{3})
        PlotPoints([varargout{3}, varargout{4}], [],  'linestyle', '-')
    end
end
