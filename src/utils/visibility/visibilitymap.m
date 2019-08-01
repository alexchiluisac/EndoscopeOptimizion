function seenVertices = visibilitymap(viewPoint, approachVec, meModel, osModel)
%% WE'LL WRITE THE DOCUMENTATION LATER
%  03/14/2019 still no documentation, dammit

  osFaces = osModel.Faces + length(meModel.Vertices);
  faces = [meModel.Faces; osFaces]';
  vertices = [meModel.Vertices; osModel.Vertices]';

%   faces = anatomyModel.Faces';
%   vertices = anatomyModel.Vertices';
  
  % Cast rays from the viewPoint to each of the centroids
  rays = bsxfun(@minus, meModel.Vertices', viewPoint);
  
  % Convert rays into unit vectors
  raysu = zeros(size(rays, 1), size(rays, 2));
  
  for k = 1 : length(rays)
      raysu(:,k) =  rays(:,k) / norm(rays(:,k));
  end
  
  % Make as many copies of approachVec as the number of rays we generated
  approachVecRep = repmat(approachVec, 1, length(raysu));
  
  % See what rays fall within the "field of view" of the camera
  product = sum(approachVecRep .* raysu);
  FOV =  15 * pi / 180;
  seenVertices = (product > cos(FOV / 2));
  seenVertices = visualrange(viewPoint, vertices, double(seenVertices), faces-1);
  
%   seenFaces = seenVertices(faces);
%   seenFaces = sum(seenFaces, 1);
%   seenFaces(seenFaces < 3) = 0;
%   seenFaces(seenFaces == 3) = 1;
  
end
