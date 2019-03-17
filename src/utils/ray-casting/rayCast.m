% p point sampled by RRT
% larynxModel
% approachVec third column of rotation matrix returned by RRT

function intersect = rayCast(p, larynxModel, approachVec)

% generate rays in random directions
%n = 3000;
%r = randn(3,n); % Use a large n
%r = bsxfun(@rdivide,r,sqrt(sum(r.^2,1))) + p;
rays = bsxfun(@minus, larynxModel.vertices', p);

% convert rays into unit vectors
for k = 1 : length(rays)
  rays(:,k) =  rays(:,k) / norm(rays(:,k));
end
        
% make as many copies of approachVec as the number of rays we generated
approachVec = repmat(approachVec, 1, length(rays));
        
% see what rays fall within the "field of view" of the laser and calculate a table of mesh triangles that are "seen"
product = sum(approachVec .* rays);
FOV = 32.44 / 1000; % divergence angle of the laser beam 32.44 millirads
%FOV = 10 * pi / 180;
seen_map = (product > cos(FOV / 2));
        
intersect = visualrange(p, larynxModel.vertices', ...
           double(seen_map), larynxModel.faces'-1);  
end
