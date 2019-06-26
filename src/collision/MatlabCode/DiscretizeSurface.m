function Spts = DiscretizeSurface(S)
nsamps = 100;
res = 1/(nsamps-1);

K = convhull(S.');

L = 0:res:1;
ct = 1;
for ll = 1:size(K, 1)
    tri = S(:,K(ll,:));
    nsampsv = nsamps;
    for ii = 1:nsamps
        for jj = 1:nsampsv
            Spts(:, ct) = tri(:,1) + L(ii)*(tri(:,2)-tri(:,1)) + L(jj)*(tri(:,3)-tri(:,1));
            ct = ct+1;
        end
        nsampsv = nsampsv-1;
    end
end

% figureha
% trisurf(convhull(S.'), S(1,:), S(2,:), S(3,:), 'facealpha', '0.3', 'edgealpha', '1', 'facecolor', 'g')
% PlotPoints(Spts)
end