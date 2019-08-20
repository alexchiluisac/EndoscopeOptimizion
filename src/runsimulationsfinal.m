for ii = 1 : 3
    experiment{ii}.u = 0.84;
    experiment{ii}.h = 0.34;
    experiment{ii}.n = ii+2;
    experiment{ii}.minAdvancement = 3-ii;
end

parfor ii = 1 : 3
    u = experiment{ii}.u;
    h = experiment{ii}.h;
    n = experiment{ii}.n;
    minAdvancement = experiment{ii}.minAdvancement;
    
    estimateReachableWorkspace(u, h, n, minAdvancement);
end