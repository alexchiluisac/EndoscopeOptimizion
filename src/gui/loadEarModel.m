function [earModel, T, earModelRayCast, earSegmentation] = loadEarModel(id)
% Returns the desired ear model, plus a H.T. matrix to transform from the 
%  world reference frame to that of the model (CT image)

%% Add 'utils' folder to path - it contains the "meshread" function
% addpath('utils')

%% Read parameters from `configurations.txt`
fname = fullfile('anatomical-models', 'configurations.txt');
fid = fopen(fname);
text = textscan(fid, '%s %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f');
fclose(fid);

configurations = cell2mat(text(2:end));
line_no = find(strcmp(text{1}, id));

path       = fullfile('anatomical-models', text{1}(line_no));
path       = path{:}; % converts cell to char
image_size   = configurations(line_no, 1:3);
voxel_size   = configurations(line_no, 4:6);
entry_point  = configurations(line_no, 7:9);
tip_base     = configurations(line_no, 10:12);
%target_point = configurations(line_no, 13:15);


%% Load Meshes and Masks
me_meshfile      = fullfile(path, 'me.mesh');
me_mesh          = meshread(me_meshfile);
ossicle_meshfile = fullfile(path, 'ossicle.mesh');
ossicle_mesh     = meshread(ossicle_meshfile);

%% Tweak mesh format to make it MATLAB-ready
mmesh.Faces = me_mesh.triangles' + 1;
mmesh.Vertices = bsxfun(@times, me_mesh.vertices', voxel_size);
mmesh.FaceVertexCData = ones(size(mmesh.Vertices, 1), 1);
%mmesh.FaceColor = 'flat';
mmesh.EdgeColor = 'none';
mmesh.FaceAlpha = 0.4 ;

omesh.Faces = ossicle_mesh.triangles' + 1;
omesh.Vertices = bsxfun(@times, ossicle_mesh.vertices', voxel_size);
omesh.FaceVertexCData = ones(size(omesh.Vertices, 1), 1);
omesh.FaceColor = 'flat';
omesh.FaceAlpha = 0.4 ;

earModel.me = mmesh;
earModel.os = omesh;

%% Now calculate the H.T. matrix
P = entry_point' - [0 0 0]';
u1 = (tip_base' - entry_point') / norm(tip_base' - entry_point');
u2 = [0 0 1]; % unit vector along z 
R = calculateRotation(u2, u1);
T = eye(4);
T(1:3,1:3) = R;
T(1:3,4) = P;

%% Generate model for ray-casting algorithm
ostriangles   = ossicle_mesh.triangles + me_mesh.numverts;
earModelRayCast.bothtriangles = [me_mesh.triangles, ostriangles]';
bothvertices = [me_mesh.vertices, ossicle_mesh.vertices];
earModelRayCast.bothvertices = bsxfun(@times, bothvertices', voxel_size);
earModelRayCast.meMeshNumVerts = me_mesh.numverts;

%% Load segmentation
%load(fullfile(path, 'record.mat'), 'recordnew');
%earSegmentation = recordnew(:);
earSegmentation = 0;
end