classdef OcTree < handle
% OcTree point decomposition in 3D
%    OcTree is used to create a tree data structure of bins containing 3D
%    points. Each bin may be recursively decomposed into 8 child bins.
%
%    OT = OcTree(PTS) creates an OcTree from an N-by-3 matrix of point
%    coordinates.
%
%    OT = OcTree(...,'PropertyName',VALUE,...) takes any of the following
%    property values:
%
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
%
%    Example 1: Decompose 200 random points into bins of 20 points or less,
%             then display each bin with its points in a separate colour.
%        pts = (rand(200,3)-0.5).^2;
%        OT = OcTree(pts,'binCapacity',20);        
%        figure
%        boxH = OT.plot;
%        cols = lines(OT.BinCount);
%        doplot3 = @(p,varargin)plot3(p(:,1),p(:,2),p(:,3),varargin{:});
%        for i = 1:OT.BinCount
%            set(boxH(i),'Color',cols(i,:),'LineWidth', 1+OT.BinDepths(i))
%            doplot3(pts(OT.PointBins==i,:),'.','Color',cols(i,:))
%        end
%        axis image, view(3)
%
%    Example 2: Decompose 200 random points into bins of 10 points or less,
%             shrunk to minimallly encompass their points, then display.
%        pts = rand(200,3);
%        OT = OcTree(pts,'binCapacity',10,'style','weighted');
%        OT.shrink
%        figure
%        boxH = OT.plot;
%        cols = lines(OT.BinCount);
%        doplot3 = @(p,varargin)plot3(p(:,1),p(:,2),p(:,3),varargin{:});
%        for i = 1:OT.BinCount
%            set(boxH(i),'Color',cols(i,:),'LineWidth', 1+OT.BinDepths(i))
%            doplot3(pts(OT.PointBins==i,:),'.','Color',cols(i,:))
%        end
%        axis image, view(3)
%
%
% OcTree methods:
%     shrink            - Shrink each bin to tightly encompass its children
%     query             - Ask which bins a new set of points belong to.
%     Plot              - Plots bin bounding boxes to the current axes.
%     Transform         - Transforms the octree, given a homogeneous transformation. 
%     Translate         - Translates the octree, given a vector. 
%     Rotate            - Rotates the octree, given a rotation matrix. 
%     Prune             - Removes empty bins
%
% OcTree properties:
%     Points            - The coordinate of points in the decomposition.
%     PointBins         - Indices of the bin that each point belongs to.
%     BinCount          - Total number of bins created.
%     BinBoundaries     - BinCount-by-6 [MIN MAX] coordinates of bin edges.
%                          (This property is erased when the octree is not axis aligned.)
%     BinCorners        - 3x8xnumPts matrix containing the coordinates of bin corners.
%     BinDepths         - The # of subdivisions to reach each bin.
%     BinCenters        - The centers of each bin
%     BinIDs            - Each Bin's unique identifier
%     MaxBinDepth       - The numerical value of the deepest level of the octree
%     BinParents        - Indices of the bin that each bin belongs to.
%     Properties        - Name/Val pairs used for creation (see help above)
%
% See also qtdecomp.

%   OcTree Class was Created by Sven Holcombe.
%   1.0     - 2013-03 Initial release
%   1.1     - 2013-03 Added shrinking bins and allocate/deallocate space
%
%   Please post comments to the FEX page for this entry if you have any
%   bugs or feature requests.
% 
% Modified heavily by Michael Siebold 3/2017
%   Added Functionality
%       Oriented Bounding Boxes (OBB)
%       Voxelized Octrees
%       PCA Aligned Octrees
%       BinCenters
%       BinCorners
%       Transform
%       Rotate
%       Translate
%       Plot (deleted and rewritten)
%       Prune
%       
    properties
        Points
        PointBins
        BinCount
        BinBoundaries
        BinCorners
        BinDepths
        BinCenters
        BinIDs
        MaxBinDepth
        BinParents = zeros(0,1)
        Properties
    end
    
    methods
        
        function this = OcTree(pts,varargin)
            % This is the OcTree header line
            validateattributes(pts,{'numeric'},...
                {'real','finite','nonnan','ncols', 3},...
                mfilename,'PTS')
            
            numPts = size(pts,1);
            
            % Allow custom setting of Properties
            IP = inputParser;
            IP.addParameter('binCapacity', ceil(numPts)/10);
            IP.addParameter('maxDepth', inf);
            IP.addParameter('maxSize', inf);
            IP.addParameter('minSize', 1000 * eps);
            IP.addParameter('style', 'equal');
            IP.addParameter('Aligned','PCA');
            IP.addParameter('Voxelized', false);
            IP.parse(varargin{:});
            this.Properties = IP.Results;
            
            this.Points = pts;
            
            if this.Properties.Voxelized
                this.Properties.binCapacity = 1;
                this.Properties.Aligned = 'Axis';
                this.Properties.maxSize = 1;
                % Align points with PCA if desired before first bin is initialized.
                [this, H] = PCAAlignPoints(this);
                
                % Extra padding is required to ensure the voxelization is
                % correct. This routine splits the padding as evenly as
                % possible around the voxels.
                mn = min(this.Points(:))-0.5;
                mx = max(this.Points(:))+0.5;
                buffer = (2^ceil(log2(mx-mn)) - (mx-mn))/2;
                this.BinBoundaries = [repmat(mn - floor(buffer),1, 3), repmat(mx + ceil(buffer), 1, 3)];
            else
                % Align points with PCA if desired before first bin is initialized.
                [this, H] = PCAAlignPoints(this); 
                this.BinBoundaries = [min(this.Points,[],1) max(this.Points,[],1)];
            end
            
            % Initialise a single bin surrounding all given points
            this.PointBins = ones(numPts,1);
            this.BinDepths = 0;
            this.BinParents(1) = 0;
            this.BinCount = 1;
            
            % Return on empty or trivial bins
%             if numPts<2, return; end
            
            % Start dividing!
            this.preallocateSpace;
            this.divide(1);
            this.deallocateSpace;
            this.BinIDs(1) = 1;
            this.MaxBinDepth = max(this.BinDepths);
            this.BinCenters = BoxCenters(this.BinBoundaries);
            this.BinCorners = PrepareBinCorners(this);
            this = this.PCAAlignOctree(H);
            this.Prune;
        end
        
        % MATLAB performs better if arrays that grow are initialised,
        % rather than grown during a loop. These two functions do just that
        % before and after the identification of new beens.
        function preallocateSpace(this)
            numPts = size(this.Points,1);
            numBins = numPts;
            if isfinite(this.Properties.binCapacity)
                numBins = ceil(2*numPts/this.Properties.binCapacity);
            end
            this.BinDepths(numBins) = 0;
            this.BinParents(numBins) = 0;
            this.BinBoundaries(numBins,1) = 0;
        end
        function deallocateSpace(this)
            this.BinDepths(this.BinCount+1:end) = [];
            this.BinParents(this.BinCount+1:end) = [];
            this.BinBoundaries(this.BinCount+1:end,:) = [];
        end
        
        function divide(this, startingBins)
            % Loop over each bin we will consider for division
            for i = 1:length(startingBins)
                binNo = startingBins(i);
                
                % Prevent dividing beyond the maximum depth
                if this.BinDepths(binNo)+1 >= this.Properties.maxDepth
                    continue;
                end
                
                % Prevent dividing beyond a minimum size                
                thisBounds = this.BinBoundaries(binNo,:);
                binEdgeSize = diff(thisBounds([1:3;4:6]));
                minEdgeSize = min(binEdgeSize);
                maxEdgeSize = max(binEdgeSize);
                if minEdgeSize < this.Properties.minSize
                    continue;
                end
                
                % There are two conditions under which we should divide
                % this bin. 1: It's bigger than maxSize. 2: It contains
                % more points than binCapacity.
                oldCount = this.BinCount;
                if nnz(this.PointBins==binNo) > this.Properties.binCapacity
                    this.divideBin(binNo);
                    this.divide(oldCount+1:this.BinCount);
                    continue;
                end
                if maxEdgeSize>this.Properties.maxSize
                    this.divideBin(binNo);
                    this.divide(oldCount+1:this.BinCount);
                    continue;
                end
            end
        end
        
        function divideBin(this,binNo)
            % Gather the new points (a bit more efficient to copy once)
            binPtMask = this.PointBins==binNo;
            thisBinsPoints = this.Points(binPtMask,:);
            
            % Get the old corner points and the new division point
            oldMin = this.BinBoundaries(binNo,1:3);
            oldMax = this.BinBoundaries(binNo,4:6);
            if strcmp('weighted',this.Properties.style) && any(binPtMask)
                newDiv = mean(thisBinsPoints,1);
            else
                newDiv = mean([oldMin; oldMax], 1);
            end
            
            % Build the new boundaries of our 8 subdivisions
            minMidMax = [oldMin newDiv oldMax];
            newBounds = minMidMax([...
                1 2 3 4 5 6;
                1 2 6 4 5 9;
                1 5 3 4 8 6;
                1 5 6 4 8 9;
                4 2 3 7 5 6;
                4 2 6 7 5 9;
                4 5 3 7 8 6;
                4 5 6 7 8 9]);
            
            % Determine to which of these 8 bins each current point belongs
            binMap = cat(3,[0 0 0],[0 0 1],[0 1 0],[0 1 1],...
                [1 0 0],[1 0 1],[1 1 0],[1 1 1]);
            gtMask = bsxfun(@gt, thisBinsPoints, newDiv);
            [~,binAssignment] = max(all(bsxfun(@eq,gtMask,binMap),2),[],3);
            % [~, binAssignment] = ismember(gtMask,binMap,'rows'); % A little slower than above.
            
            % Make the new bins and reassign old points to them
            newBinInds = this.BinCount+1:this.BinCount+8;
            
            this.BinIDs(newBinInds) = newBinInds; %adding a bin ID number
            
            this.BinBoundaries(newBinInds,:) = newBounds;
            this.BinDepths(newBinInds) = this.BinDepths(binNo)+1;
            this.BinParents(newBinInds) = binNo;
            this.PointBins(binPtMask) = newBinInds(binAssignment);
            this.BinCount = this.BinCount + 8;
        end
        
        function shrink(this)
            % Shrink all bins to bound only the points they contain
            % WARNING: this operation creates gaps in the final space not
            % covered by a bin. Only shrink OcTree structures when you only
            % intend to use the points used to create the tree to query the
            % tree space.
            binChildren = arrayfun(@(i)find(this.BinParents==i),1:this.BinCount,'Un',0)';
            binIsLeaf = cellfun(@isempty, binChildren);
            for i = find(binIsLeaf(:))'
                binShrink_recurse(i, true)
            end
            
            function binShrink_recurse(binNo, isLeafBin)
                % Build a list of all points that fall within one of the
                % bins to be checked, and the list of which point falls in
                % which bin.
                oldBoundaryMin = this.BinBoundaries(binNo,1:3);
                oldBoundaryMax = this.BinBoundaries(binNo,4:6);
                if isLeafBin
                    % Shrink bin based on child POINTS
                    ptsMask = this.PointBins==binNo;
                    if ~any(ptsMask)
                        % No points, shrink the bin to infinitely small
                        proposedBoundaries = [oldBoundaryMin oldBoundaryMin];
                    else
                        pts = this.Points(ptsMask,:);
                        proposedBoundaries = [...
                            max([oldBoundaryMin; min(pts,[],1)]) ...
                            min([oldBoundaryMax; max(pts,[],1)])];
                    end
                else
                    % Shrink bin based on child BINS
                    childBoundaries = this.BinBoundaries(binChildren{binNo},:);
                    proposedBoundaries = [min(childBoundaries(:,1:3),[],1) max(childBoundaries(:,4:6),[],1)];
                end
                
                if ~isequal(proposedBoundaries, [oldBoundaryMin oldBoundaryMax])
                    % We just shrunk the boundary. Make it official and
                    % check the parent
                    this.BinBoundaries(binNo,:) = proposedBoundaries;
                    parentBin = this.BinParents(binNo);
                    if parentBin>0
                        binShrink_recurse(parentBin, false)
                    end
                end
            end
        end
        
        function binNos = query(this, newPts, queryDepth)
            % Get the OcTree bins that new query points belong to.
            %
            % BINS = OT.query(NEWPTS) searches the OcTree object OT and
            % returns an N-by-1 vector of BINS giving the bin index in
            % which each of the N points in NEWPTS is contained. For any
            % query points outside all bins in OT, the index -1 is
            % returned.
            %
            % BINS = OT.query(NEWPTS,DEPTH) restricts the search to DEPTH
            % levels in the OT bin tree. Note that the first bin
            % (containing all other bins in OT) has DEPTH = 1.

            if nargin<3
                queryDepth = max(this.BinDepths);
            end
            
            numPts = size(newPts,1);
            newPts = permute(newPts,[3 2 1]);
            binNos = ones(numPts,1)*-1;
                        
            binChildren = arrayfun(@(i)find(this.BinParents==i),1:this.BinCount,'Un',0)';
            binIsLeaf = cellfun(@isempty, binChildren);
            ptQuery_recurse(1:numPts, this.BinParents==0, 0)
            
            function ptQuery_recurse(newIndsToCheck_, binsToCheck, depth)
                % Build a list of all points that fall within one of the
                % bins to be checked, and the list of which point falls in
                % which bin.
                boundsToCheck = this.BinBoundaries(binsToCheck,:);
                [ptInBounds, subBinNo] = max(all(...
                    bsxfun(@ge, newPts(:,:,newIndsToCheck_), boundsToCheck(:,1:3)) & ...
                    bsxfun(@le, newPts(:,:,newIndsToCheck_), boundsToCheck(:,4:6))...
                    ,2),[],1);
            
                if ~all(ptInBounds)
                    % Special case usually when depth=0, where a point may
                    % fall outside the bins entirely. This should only
                    % happen once so let's fix it once and let subsequent
                    % code rely on all points being in bounds
                    binNos(newIndsToCheck_(~ptInBounds)) = -1;
                    newIndsToCheck_(~ptInBounds) = [];
                    subBinNo(~ptInBounds) = [];
                end
                binNosToAssign = binsToCheck(subBinNo);
                newIndsToAssign = newIndsToCheck_;
                binNos(newIndsToAssign) = binNosToAssign;
                
                % Allow a free exit when we reach a certain depth
                if depth>=queryDepth
                    return;
                end
                
                % Otherwise, for all of the points we just placed into
                % bins, check which of the children of those bins those
                % same points fall into
                [unqBinNos, ~, unqGrpNos] = unique(binNosToAssign);
                for i = 1:length(unqBinNos)
                    thisPtMask = unqGrpNos==i;
                    if ~binIsLeaf(unqBinNos(i))
                        ptQuery_recurse(newIndsToCheck_(thisPtMask), binChildren{unqBinNos(i)}, depth+1)
                    end
                end
                
            end
        end
        
        function h = Plot(this, BinDepth, h, varargin)
            % OcTree.plot plots bin bounding boxes of an OcTree object
            %
            % H = OT.plot('name',value,...) allows you to specify any
            % properties of the bounding box lines that you would normally
            % supply to a plot(...,'name',value) command, and returns plot
            % object handles (one per bin) to H.
            
            if nargin > 1
                idxs = [];
                for ii = 1:length(BinDepth)
                    idxs = [idxs, find(this.BinDepths == BinDepth(ii))]; %#ok<AGROW>
                end
            else
                idxs = 1:this.BinCount;
            end
            
            if nargin < 2
                h = figure;
                hold on
                axis equal
                set(gcf, 'color', [1,1,1])
            end
            
            for ii = idxs
                this.PlotOABB(ii, varargin{:})
            end
            PlotPoints(this.Points)
        end
        
        function varargout = PlotOABB(OT, ii, varargin)
            
            hg = hggroup;
            map = hsv(OT.MaxBinDepth+1);
            
            faces = [1,2,4,3;1,5,6,2;5,6,8,7;7,3,4,8;2,6,8,4;1,5,7,3];
            
            PatchChars = {'edgecolor', map(OT.BinDepths(ii)+1,:)}; % 'facecolor', map(OT.BinDepths(ii)+1,:)
%             PointChars = {'linestyle', 'none', 'marker', '*', 'markersize', 5, 'color', map(OT.BinDepths(ii)+1,:), 'parent', hg};
%             PlotPoints(OT.BinCorners(:,:,ii).', [], varargin{:}, PointChars{:})
            if isempty(varargin)
                patch('vertices', OT.BinCorners(:,:,ii).', 'faces', faces, 'facealpha', 0, PatchChars{:}, 'parent', hg)
            else
                patch('vertices', OT.BinCorners(:,:,ii).', 'faces', faces, 'facealpha', 0, varargin{:}, 'parent', hg)
            end
            if nargout>0
                varargout{1} = hg;
            end
        end
        
        function OT = Transform(OT, H)
            OT.Points = (H(1:3,1:3)*OT.Points.' + repmat(H(1:3,4), 1, size(OT.Points, 1))).';
            OT.BinCenters = (H(1:3,1:3)*OT.BinCenters.' + repmat(H(1:3,4), 1, OT.BinCount)).';
            OT.BinBoundaries = [];
            OT.BinCorners = TransformPoints(H, OT.BinCorners);
        end
        
        function OT = Rotate(OT, R)
            OT.Points = (R*OT.Points.').';
            OT.BinCenters = (R*OT.BinCenters.').';
            OT.BinBoundaries = [];
            OT.BinCorners = TransformPoints(R, OT.BinCorners);
        end
        
        function OT = Translate(OT, T)
            
            OT.Points = OT.Points + repmat(T(:).', size(OT.Points, 1), 1);
            OT.BinCenters = OT.BinCenters + repmat(T(:).', size(OT.BinCenters, 1), 1);
            OT.BinBoundaries = [];
            OT.BinCorners = TransformPoints(T, OT.BinCorners);
        end
        
        function [OT, H] = PCAAlignPoints(OT)
            H = [];
            if strcmpi(OT.Properties.Aligned, 'PCA')
                H = [[pca(OT.Points), mean(OT.Points, 1).']; [0,0,0,1]];
                % Demean and rotate the points.
                PointsDM = OT.Points.' - repmat(H(1:3,4), 1, size(OT.Points, 1));
                OT.Points = (H(1:3,1:3).'*PointsDM).';
            end
        end
        
        function OT = PCAAlignOctree(OT, H)
            if strcmpi(OT.Properties.Aligned, 'PCA')
                OT = OT.Transform(H);
            end
        end
        
        function OT = Prune(OT)
            OT = RemoveEmptyBins(OT);
            OT = RemoveChildlessBins(OT); 
        end
        
        function OT = RemoveBins(OT, RemoveIds)
            % untested...should work.
            
            Remove = false(1, length(OT.PointBins));
            Remove(RemoveIds) = true;
            
            IDShift = cumsum(Remove);
            ParentsShift = [0, IDShift(OT.BinParents(2:end))];
            
            OT.PointBins = OT.PointBins - IDShift(OT.PointBins).';
            
            OT.BinCorners = OT.BinCorners(:,:,~Remove);
            OT.BinDepths = OT.BinDepths(~Remove);
            OT.BinIDs = OT.BinIDs(~Remove) - IDShift(~Remove);
            OT.BinParents = OT.BinParents(~Remove) - ParentsShift(~Remove);
            OT.BinCenters = OT.BinCenters(~Remove,:);
            OT.BinCount = sum(~Remove);
        end
    end
end

function BinCorners = PrepareBinCorners(OT)
BinCorners = ...
    permute(cat(3,[OT.BinBoundaries(:,1).';OT.BinBoundaries(:,2).';OT.BinBoundaries(:,3).'],...
                  [OT.BinBoundaries(:,1).';OT.BinBoundaries(:,2).';OT.BinBoundaries(:,6).'],...
                  [OT.BinBoundaries(:,1).';OT.BinBoundaries(:,5).';OT.BinBoundaries(:,3).'],...
                  [OT.BinBoundaries(:,1).';OT.BinBoundaries(:,5).';OT.BinBoundaries(:,6).'],...
                  [OT.BinBoundaries(:,4).';OT.BinBoundaries(:,2).';OT.BinBoundaries(:,3).'],...
                  [OT.BinBoundaries(:,4).';OT.BinBoundaries(:,2).';OT.BinBoundaries(:,6).'],...
                  [OT.BinBoundaries(:,4).';OT.BinBoundaries(:,5).';OT.BinBoundaries(:,3).'],...
                  [OT.BinBoundaries(:,4).';OT.BinBoundaries(:,5).';OT.BinBoundaries(:,6).']),...
                  [1,3,2]);
end

function centers = BoxCenters(bounds)
centers = [bounds(:,1)+bounds(:,4), bounds(:,2)+bounds(:,5), bounds(:,3)+bounds(:,6)]/2;
end

function OT = RemoveEmptyBins(OT)
% remove empty bins at maximum bin depth
Occupied = false(1,OT.BinCount);
Occupied(OT.PointBins) = true;
NotMaxBinDepth = OT.BinDepths ~= OT.MaxBinDepth;
Keep = Occupied | NotMaxBinDepth;
OT = KeepBins(OT, Keep);
end

function OT = RemoveChildlessBins(OT)

% remove childless bins
for ii = OT.MaxBinDepth-1:-1:0
    ParentsList = unique(OT.BinParents);
    Parents = false(1,OT.BinCount);
    Parents(ParentsList(2:end)) = true;
    Parents(1) = true; % keep the 0 parent bin (the original, largest bin)
    
    BinsNotAtCurrentDepth = OT.BinDepths < ii | OT.BinDepths > ii;
    
    Keep = Parents | BinsNotAtCurrentDepth;
    OT = KeepBins(OT, Keep);
end
end

function OT = KeepBins(OT, Keep)

IDShift = cumsum(~Keep);
ParentsShift = [0, IDShift(OT.BinParents(2:end))];

OT.PointBins = OT.PointBins - IDShift(OT.PointBins).';

OT.BinCorners = OT.BinCorners(:,:,Keep);
OT.BinDepths = OT.BinDepths(Keep);
OT.BinIDs = OT.BinIDs(Keep) - IDShift(Keep);
OT.BinParents = OT.BinParents(Keep) - ParentsShift(Keep);
OT.BinCenters = OT.BinCenters(Keep,:);
OT.BinCount = sum(Keep);
end


