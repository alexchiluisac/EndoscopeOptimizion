addpath('cost-functions');

alpha = [0 pi];

v = zeros(1,length(alpha));
seenMaps = {};

parfor i = 1 : length(alpha)
    [v(i), seenMaps{i}] = visiblesurface(alpha(i));
end

figure
scatter(alpha, v, 20, 'filled');
xlabel('alpha[rad]');
ylabel('Visible Surface[mm^2]');
ylim([0 max(v)]);
grid on

figure
path = fullfile('..', 'anatomical-models', 'synthetic-model-finer.stl');
[vertices, faces, ~, ~] = stlRead(path);
earModel.vertices = vertices;
earModel.faces = faces;

subplot(1,2,1);

for ii = 1 : 2
   subplot(1,2,ii);
   stlPlot(earModel.vertices, earModel.faces, num2str(alpha(ii)), seenMaps{ii});
end