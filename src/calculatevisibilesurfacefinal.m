%visibility = zeros(1,7);

for ii = 3 % : 9
    fid = fopen(fullfile('..', 'anatomical-models', 'configurations.txt'));
    text = textscan(fid, '%s %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f');
    fclose(fid);
    
    configurations = cell2mat(text(2:end));
    line_no = find(strcmp(text{1}, modelID));
    
    image_size   = configurations(line_no, 1:3);
    voxel_size   = configurations(line_no, 4:6);
    
    load([num2str(ii) '-simulation.mat']);
    path = fullfile('..', 'anatomical-models', modelID);
    load(fullfile(path, 'record.mat'));
    
    % Read the Raw Meshes from file
    pathMe = fullfile(path, 'me.mesh');
    pathOs = fullfile(path, 'ossicle.mesh');
    rawMeMesh = meshread(pathMe);
    rawOsMesh = meshread(pathOs);
    
    % Convert the raw meshes into objects that can be passed
    % to the `patch' function
    meMesh.Faces = rawMeMesh.triangles' + 1;
    meMesh.Vertices = bsxfun(@times, rawMeMesh.vertices', voxel_size);
    meMesh.Vertices = meMesh.Vertices .* 1e-3;
    
    osMesh.Faces = rawOsMesh.triangles' + 1;
    osMesh.Vertices = bsxfun(@times, rawOsMesh.vertices', voxel_size);
    osMesh.Vertices = osMesh.Vertices .* 1e-3;
    
    meMesh.FaceVertexCData = ones(size(meMesh.Vertices, 1), 1);
    meMesh.LineStyle = 'none';
    meMesh.FaceColor = 'flat';
    meMesh.FaceAlpha = 0.4 ;
    
    osMesh.FaceVertexCData = ones(size(osMesh.Vertices, 1), 1);
    osMesh.FaceColor = 'flat';
    osMesh.FaceAlpha = 0.4 ;
    
    
    % Calculate the visual range
    seenMap = false(rawMeMesh.numverts, length(pList));
    
    parfor jj = 1 : 2%length(pList)
        seenMap(:,jj) = visibilitymap(pList(:,jj), aList(:,jj), meMesh, osMesh);
    end
    
    seenMap = sum(seenMap, 2);
    seenMap(seenMap > 1) = 1;
    
    visibility{ii} = area_stat(recordnew(:), seenMap(:));
end