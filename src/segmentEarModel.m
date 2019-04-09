path = fullfile('..', 'anatomical-models', 'synthetic-model-finer-cropped.stl');
[vertices, faces, ~, ~] = stlRead(path);
earModel.vertices = vertices;
earModel.faces = faces;

ROImap = ones(1, length(faces));

for ii = 1 : length(faces)
    face = faces(ii,:);
    
    for jj = 1 : 3
        p = face(jj);
        vertex = vertices(p,:);
        
        if vertex(1) > 20 || vertex(3) > 5.5 || vertex(3) < 4
            ROImap(ii) = 0;
            break;
        end
    end   
end


stlPlot(earModel.vertices, earModel.faces, 'Segmentation Test', ROImap);

save('../anatomical-models/mesegmentation.mat', 'ROImap');