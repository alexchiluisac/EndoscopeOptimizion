ObjectiveFunction = @visiblesurface2;
n = 8; % number of cutouts;

initialDesign = zeros(1,4*n);
initialDesign(1:n) = ones(1,n) * 1.20 * 1e-3; % cutout width
initialDesign(n+1:2*n) = ones(1,n) * 1.2 * 1e-3; % length of uncut sections
initialDesign(2*n+1:3*n) = ones(1,n) * 0.19 * 1e-3; % cutout height
initialDesign(3*n+1:4*n) = zeros(1,n); % cutout orientation

lb = zeros(1,4*n);
lb(1:n) = ones(1,n) * 1.10 * 1e-3; % cutout width
lb(n+1:2*n) = ones(1,n) * 0.5 * 1e-3; % length of uncut sections
lb(2*n+1:3*n) = ones(1,n) * 0.10 * 1e-3; % cutout height
lb(3*n+1:4*n) = zeros(1,n); % cutout orientation

ub = zeros(1,4*n);
ub(1:n) = ones(1,n) * 1.30 * 1e-3; % cutout width
ub(n+1:2*n) = ones(1,n) * 1.8 * 1e-3; % length of uncut sections
ub(2*n+1:3*n) = ones(1,n) * 0.29 * 1e-3; % cutout height
ub(3*n+1:4*n) = 2*pi*ones(1,n); % cutout orientation

options = optimoptions('simulannealbnd','PlotFcns',...
          {@saplotbestx,@saplotbestf,@saplotx,@saplotf});

rng default % For reproducibility

[x,fval,exitFlag,output] = simulannealbnd(ObjectiveFunction, initialDesign, lb, ub, options);