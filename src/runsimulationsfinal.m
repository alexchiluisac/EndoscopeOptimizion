for ii = 1 : 5
    experiment{ii}.u = 0.84;
    experiment{ii}.h = 0.34;
    experiment{ii}.n = ii;
    experiment{ii}.minAdvancement = 0;
end

parfor ii = 1 : 5
    u = experiment{ii}.u;
    h = experiment{ii}.h;
    n = experiment{ii}.n;
    minAdvancement = experiment{ii}.minAdvancement;
    estimateReachableWorkspace(u, h, n, minAdvancement);
end