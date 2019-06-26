% CollisionDetection.m Examples
% Michael Siebold 
% 3/22/2017

% Load an stl of a structure
load('DemoSpace.mat', 'FN')

% create two random shapes
S1 = rand(3,9)-0.5;
S2 = rand(3,9)-0.5 + repmat([1;2;3], 1, 9);

% test those two shapes for a collision (probably won't collide)
CollisionDetection(S1, S2, 'Plotting', true)

%% Show the minimum distance calculation for S1 and S2
[Collision, MD, cpS1, cpS2] = CollisionDetection(S1, S2, 'Plotting', true)

%% Two objects that (probably) will collide
S3 = rand(3,9)-0.5 + repmat(0.1*[1;1;0], 1, 9);
% test two shapes for a collision (probably will collide)
CollisionDetection(S1, S3, 'Plotting', true)

%% Create and display an Octree for the facial nerve
OT_FN = OcTree(FN.vertices.', 'Aligned', 'PCA', 'maxSize', 2);
OT_FN.Plot

%% Test a structure against the facial nerve
FNMean = mean(FN.vertices, 2); % find a point near the facial nerve

% create a structure near the facial nerve (Probably not colliding)
S4 = 5*(rand(3,9)-0.5) + repmat(FNMean + 5*[1;1;0], 1, 9);
CollisionDetection(OT_FN, S4, 'Plotting', true)

%% Model a drill shaft and test for collision

Shaft1 = ModelShaft(AddVectors(FNMean, -5*ones(3,1)), FNMean, 1);
CollisionDetection(Shaft1, OT_FN, 'Plotting', true)

%% Model a drill shaft and test for collision and plot the results of each test

Shaft2 = ModelShaft(AddVectors(FNMean, -5*ones(3,1)), AddVectors(FNMean, [-2;2;0]), 1);
CollisionDetection(Shaft2, OT_FN, 'Plotting', true, 'IntermediatePlotting', true)
% evalin('base', 'PlotPoints(OT_FN.Points)') % use this to visualize the
                                             % facial nerve while plotting 
                                             % the collision detection

%% CollisionDetection can create the octree given an stl

STL1.vertices = DiscretizeSurface(S4);
% Pass the collection of points in 'STL1.vertices' directly into 
% 'CollisionDetection' and have 'CollisionDetection' create the OcTree 
% automatically.
CollisionDetection(STL1, OT_FN, 'Plotting', true, 'Object1maxSize', 1)

% maxSize is an important parameter to set. If it is not small enough some
% points may be contained in bins that are not at the max depth.

%% OcTree can create an octree where the deepest level bins correspond to voxels.

Region = make_region(25, 25, 25);
OT_Target = OcTree(ExtractSubscripts(Region == 2).', 'Voxelized', true)
OT_Target.Plot
figure
hold on 
axis equal
set(gcf, 'color', [1,1,1])

OT_Target.Plot(OT_Target.MaxBinDepth)


%%
