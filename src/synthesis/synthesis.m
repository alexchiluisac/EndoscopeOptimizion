function newCurve = partitioncurve(varargin)
    %SELECTPTS Runs an algorithm that selects the dominant points in the
    %curve
    
    % Input handling
    defaultM = 3;
    defaultPlot = false;
    defaultTargetName = 'curvature';
    
    p = inputParser;
    addRequired(p, 'curve');
    addOptional(p, 'm', defaultM);
    addOptional(p, 'target', defaultTargetName);
    addParameter(p, 'plot', defaultPlot);
    parse(p, varargin{:});
    
    curve     = p.Results.curve;
    targetVarName = p.Results.target;
    m         = p.Results.m;
    plot      = p.Results.plot;
    
    if strcmp(targetVarName, 'curvature')
        var = curve.kappa;
    else
        var = curve.tau;
    end
      
    % Calculate the integral of {curvature | torsion}
    total = trapz(curve.l, var);
    
    % Now partition the curve into sections.
    % Each section should have total {curvature | torsion} == totalKappa / m
    quantum = total / m;
    
    t = zeros(1, m);
    arcIndexes = zeros(1, m);
    averageKappa = zeros(1, m);
    averageTau = zeros(1, m);
    accVar = 0;
    ii = 2;
    
    
    for jj = 1 : m
        targetVar = jj * quantum;
        kappaSegment = 0;
        tauSegment = 0;
        kk = 0;
        
        while accVar <= targetVar
            if ii == length(curve.arc)
                break;
            end
            
            accVar = accVar + ...
                trapz(curve.l(ii-1:ii), var(ii-1:ii)); 
            
            kappaSegment = kappaSegment + ...
                curve.kappa(ii);
            
            tauSegment = tauSegment + ...
                curve.tau(ii);
            
            ii = ii + 1;
            kk = kk + 1;
        end
        
        t(jj) = curve.l(ii);
        averageKappa(jj) = kappaSegment / kk;
        averageTau(jj) = tauSegment / kk;
        arcIndexes(jj) = ii;
    end
    
    dominantPts = curve.arc(:,arcIndexes);
    dominantPts = [curve.arc(:,1) dominantPts];
    
    newCurve.arc = dominantPts;
    newCurve.l   = [curve.l(1); curve.l(arcIndexes)];
    newCurve.t   = [curve.t(:,1) curve.t(:,arcIndexes)];
    newCurve.n   = [curve.n(:,1) curve.n(:,arcIndexes)];
    newCurve.b   = [curve.b(:,1) curve.b(:,arcIndexes)];
    newCurve.averageKappa = averageKappa
    newCurve.averageTau = averageTau;
    
    if plot
        figure
        plot3(curve.arc(1,:), curve.arc(2,:), curve.arc(3,:), 'LineWidth',3,'Color',(1/256)*[255 128 0]);

        axis equal, grid on, hold on
        triad('scale', 1e-3/2, 'linewidth', 2.5);
        scatter3(dominantPts(1,:), dominantPts(2,:), dominantPts(3,:), 80, 'filled', 'MarkerFaceColor', (1/256)*[96 96 96]);
        xlabel('X [m]'), ylabel('Y [m]'), zlabel('Z [m]')
        legend({'Original Curve', 'Dominant Points'});
        view(0.26, 20.5)
        title('Partitioned Curve');
        set(gca,'FontSize',16);
    end
end
