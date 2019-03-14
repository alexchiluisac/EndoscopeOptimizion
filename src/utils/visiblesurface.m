function seenMap = visiblesurface(p, larynxModel, approachVec)
%% WE'LL WRITE THE DOCUMENTATION LATER

faces = larynxModel.faces;
vertices = larynxModel.vertices;

% calculate the centroids of the faces
pcentr = zeros(size(faces, 1), 3);

for ii = 1 : size(faces, 1)
    p1 = vertices(faces(ii,1), :);
    p2 = vertices(faces(ii,2), :);
    p3 = vertices(faces(ii,3), :);
    
    pcentr(ii,1) = 1/3 * (p1(1) + p2(1) + p3(1));
    pcentr(ii,2) = 1/3 * (p1(2) + p2(2) + p3(2));
    pcentr(ii,3) = 1/3 * (p1(3) + p2(3) + p3(3));
    
end

% run HPR to find the visible points
%param = [0 0.1 0.3 1 3 10];
param = [0.1];
visiblePoints = zeros(5,1);
%radiusExp = 3;

ii = 1;
for param_ii = param
    [visiblePtsIdx,~] = HPR(pcentr, p', param_ii);
    %[visiblePtsIdx,sphFlip] = HPR(pcentr, p', radiusExp);
    
    % cast rays and pick only those that are within the FOV
    rays = bsxfun(@minus, pcentr(visiblePtsIdx,:)', p);
    
    % convert rays into unit vectors
    raysu = zeros(size(rays, 1), size(rays, 2));
    
    for k = 1 : length(rays)
        raysu(:,k) =  rays(:,k) / norm(rays(:,k));
    end
    
    % make as many copies of approachVec as the number of rays we generated
    approachVecRep = repmat(approachVec, 1, length(raysu));
    
    % see what rays fall within the "field of view" of the laser and calculate a table of mesh triangles that are "seen"
    product = sum(approachVecRep .* raysu);
    %FOV = 65 / 1000; % divergence angle of the laser beam 32.44 millirads
    FOV =  120 * pi / 180;
    withinFOVMap = (product > cos(FOV / 2));
    
    % see what rays hit the target within a certain distance
    %distThreshold = 10; % [mm]
    %distances = vecnorm(rays);
    %withinDistanceMap = distances < distThreshold;
    
    seenMap = zeros(size(faces, 1), 1);
    %seenMap(visiblePtsIdx(withinFOVMap & withinDistanceMap)) = 1;
    seenMap(visiblePtsIdx(withinFOVMap)) = 1;
    seenMap = logical(seenMap);
    %visiblePoints(ii) = sum(seenMap);
    ii = ii + 1;
end
end
