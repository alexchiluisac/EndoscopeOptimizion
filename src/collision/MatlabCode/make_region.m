function [ Region ] = make_region(height,width,depth,...
    air_thickness,bone_surface_layer,wall_extent,wall_depth,show_region)
% MAKE_REGION - 3D "sink hole" with horizontal wall 
%
% MAKE_REGION(HEIGHT,WIDTH,DEPTH): Required dimensions of region to be
% made.
%
% MAKE_REGION(...,AIR_THICKNESS) gives number of contiguous pages at the
% top that contain air voxels (== 3) with no boundary voxels (== 0).
% AIR_THICKNESS must be >=1. Default == 1.
%
% MAKE_REGION(...,BONE_SURFACE_LAYER) gives the lowest page (highest layer
% on which bone to be drilled appears. Default == 2.
%
% MAKE_REGION(...,WALL_EXTENT) gives fraction of extent across the width of
% REGION that a wall of boundary voxels will extend. Default = 0.5 (halfway
% across).
%
% MAKE_REGION(...,WALL_DEPTH) gives the page on which the wall begins. It
% extends to one page short of the boundary at the bottom. Default is the
% the larger of (0.3*depth) and (AIR_THICKNESS + 2).
%
% MAKE_REGION(...,SHOW_REGION), if set equal to true, plots the region
%
% Created May 2013 by JM Fitzpatrick
boundary_voxel_value = 0;
start_voxel_value = 1;
target_voxel_value = 2;
air_voxel_value = 3;
region_type = 'uint16';
n=2;
n=n+1;if nargin<n,fprintf('Need at least %d arguments\n',n);end
n=n+1;
if nargin<n||isempty(air_thickness),air_thickness=1;
elseif air_thickness < 1
    fprintf('ERROR in make_region: AIR_THICKNESS must be >= 1\n')
    return;
end
n=n+1;if nargin<n||isempty(bone_surface_layer),bone_surface_layer=2;end
n=n+1;if nargin<n||isempty(wall_extent),wall_extent= 0.5;end
n=n+1;if nargin<n||isempty(wall_depth),wall_depth=max([floor(0.3*depth),air_thickness+2]);end
n=n+1;if nargin<n||isempty(show_region),show_region=true;end
wall_width = min([floor(wall_extent*width),width]); % 2 to avoid upper boundary

% Start with a Region full of boundary voxels:
Region = boundary_voxel_value*ones(height,width,depth,region_type);

% Replace boundary voxels in parabolic pit with target voxels:
[I1,I2,I3] = ndgrid(1:height,1:width,1:depth);
c1 = ceil(height/2);
c2 = ceil(width/2);
D2 = (I1-c1).^2 + (I2-c2).^2 + ((I3+3)/2).^2;
Region( D2 < (min([c1,c2,depth])-2)^2 ) = target_voxel_value;
Region(:,:,1) = air_voxel_value; % top layer should be air for input to make_path
Region(:,:,end) = boundary_voxel_value; % bottom layer must be boundary for input to make_path

% Add wall:
Region(ceil(height/2)-2:ceil(height/2)+1, 1:wall_width, wall_depth:end) = boundary_voxel_value; 
air_pages = 2:1+air_thickness;

% Replace boundary voxels in 1:air_thickness by air voxels:
temp = Region(:,:,1:air_thickness);
temp(temp==boundary_voxel_value) = air_voxel_value;
Region(:,:,1:air_thickness) = temp;

% Replace target voxels in 1:bone_surface_layer-1 by air voxels:
temp = Region(:,:,1:bone_surface_layer-1);
temp(temp==target_voxel_value) = air_voxel_value;
Region(:,:,1:bone_surface_layer-1) = temp;

if show_region
    for ii = 1:depth
        subplot(ceil(depth/3),3,ii);
        image(Region(:,:,ii)); title(num2str(ii));
        axis equal;
        axis tight;
        grid on;
    end
    colormap lines
%     figure
%     inspect_image(Region3d);
end

