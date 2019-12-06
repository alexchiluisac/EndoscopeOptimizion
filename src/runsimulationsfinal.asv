for ii = 1 : 10
    experiment{ii}.u = 0.84;
    experiment{ii}.h = 0.30;
    experiment{ii}.n = ii;
    experiment{ii}.minAdvancement = 8e-3;
end

parfor ii = 1 : 10
    u = experiment{ii}.u;
    h = experiment{ii}.h;
    n = experiment{ii}.n;
    minAdvancement = experiment{ii}.minAdvancement;
    estimateReachableWorkspace(u, h, n, minAdvancement);
end

calculatevisibilesurfacefinal;