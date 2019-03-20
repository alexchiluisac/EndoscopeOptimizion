function seenMap = visiblesurface(viewPoint, approachVec, anatomyModel)
%% WE'LL WRITE THE DOCUMENTATION LATER
%  03/14/2019 still no documentation, dammit

  faces = anatomyModel.faces';
  vertices = anatomyModel.vertices';
  
  % Cast rays from the viewPoint to each of the centroids
  rays = bsxfun(@minus, vertices, viewPoint);
  
  % Convert rays into unit vectors
  raysu = zeros(size(rays, 1), size(rays, 2));
  
  for k = 1 : length(rays)
      raysu(:,k) =  rays(:,k) / norm(rays(:,k));
  end
  
  % Make as many copies of approachVec as the number of rays we generated
  approachVecRep = repmat(approachVec, 1, length(raysu));
  
  % See what rays fall within the "field of view" of the camera
  product = sum(approachVecRep .* raysu);
  FOV =  90 * pi / 180;
  seenMap = (product > cos(FOV / 2));
  seenMap = visualrange(viewPoint, vertices, double(seenMap), faces-1);
end
