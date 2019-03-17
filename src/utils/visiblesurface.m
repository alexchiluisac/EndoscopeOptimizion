function seenMap = visiblesurface(viewPoint, approachVec, anatomyModel)
%% WE'LL WRITE THE DOCUMENTATION LATER
%  03/14/2019 still no documentation, dammit

  faces = anatomyModel.faces;
  vertices = anatomyModel.vertices;
  centroids = anatomyModel.centroids;

  % Initialize the visibility map
  seenMap = zeros(1, size(faces, 1));
  
  % Cast rays from the viewPoint to each of the centroids
  rays = bsxfun(@minus, centroids', viewPoint);
  
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
  withinFOVMap = (product > cos(FOV / 2));
  
  % Filter out those rays that are beyond the field of view
  withinFOVFaces = faces(withinFOVMap, :);
  withinFOVRays = raysu(:, withinFOVMap);
  
  % Now run ray-mesh intersection using the set of rays within the field of view.
  % Each ray should be intersecting at least - one face, but it may be
  % intersecting more! If the former, just set that face to "visible." If the
  % latter, sort faces by their distance from the viewpoint and then mark the
  % closest one as "visible" and the other ones as "not visible."
  
  % prepare the data structures required to run ray-triangle intersection
  vert1 = vertices(faces(withinFOVFaces,1),:);
  vert2 = vertices(faces(withinFOVFaces,2),:);
  vert3 = vertices(faces(withinFOVFaces,3),:);
  
  for ii = 1 : size(withinFOVRays, 2)
      [intersectionMap,distance,~,~,coordinates] = ...
          TriangleRayIntersection(viewPoint', withinFOVRays(:,ii)', ...
          vert1, vert2, vert3);
      
      coordinates = coordinates(intersectionMap, :);
      
      if size(coordinates, 1) > 1
          [~,idx] = min(distance(intersectionMap));
      end
      
      closestPoint = coordinates(idx, :);
      difference = bsxfun(@minus, centroids', closestPoint');
      difference = vecnorm(difference);
      
      [~,visibleFaceIdx] = min(difference);
      
      seenMap(visibleFaceIdx) = 1;
      ii
  end
  
  
end
