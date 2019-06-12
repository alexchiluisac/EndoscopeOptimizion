function [seenFaces, seenVertices] = visibilitymap(viewPoint, approachVec, anatomyModel)
%% WE'LL WRITE THE DOCUMENTATION LATER
%  03/14/2019 still no documentation, dammit

  faces = anatomyModel.faces;
  vertices = anatomyModel.vertices;
  viewPoint = viewPoint';
  % Cast rays from the viewPoint to each of the centroids
  rays = bsxfun(@minus, vertices, viewPoint);
%   size(rays)
%   length(rays)
  % Convert rays into unit vectors
  raysu = zeros(size(rays, 1), size(rays, 2));
  
  for k = 1 : length(rays)
      raysu(:,k) =  rays(:,k) / norm(rays(:,k));
  end
  
  % Make as many copies of approachVec as the number of rays we generated
  approachVecRep = repmat(approachVec, 1, length(raysu));
  
  % See what rays fall within the "field of view" of the camera
%   size(raysu)
%   size(approachVecRep)
  product = sum(approachVecRep(2) .* raysu);
  FOV =  90 * pi / 180;
  seenVertices = (product > cos(FOV / 2));
  seenVertices = visualrange(viewPoint, vertices, double(seenVertices), faces-1);
  
  seenFaces = seenVertices(faces);
  
end
