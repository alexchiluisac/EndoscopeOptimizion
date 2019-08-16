experiment{1}.u = 0.92;
experiment{1}.h = 0.17;
experiment{1}.n = 5;
experiment{1}.minAdvancement = 0;

experiment{2}.u = 0.92;
experiment{2}.h = 0.17;
experiment{2}.n = 8;
experiment{2}.minAdvancement = -3;

experiment{3}.u = 0.92;
experiment{3}.h = 0.17;
experiment{3}.n = 3;
experiment{3}.minAdvancement = 2;

for ii = 2
    u = experiment{ii}.u;
    h = experiment{ii}.h;
    n = experiment{ii}.n;
    minAdvancement = experiment{ii}.minAdvancement;
    
    testpathplanningrealanatomy(u,h,n,minAdvancement);
    %generateworkspacenoobstacles(u,h,n,minAdvancement);
end