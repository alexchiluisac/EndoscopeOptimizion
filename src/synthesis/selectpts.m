function dominantPts = partitioncurve(varargin)
    %SELECTPTS Runs an algorithm that selects the dominant points in the
    %curve
    
    % Input handling
    defaultM = 5;
    defaultPlot = false;
    
    p = inputParser;
    addRequired(p, 'curve');
    addOptional(p, 'm', defaultM);
    addParameter(p, 'plot', defaultPlot);
    parse(p, varargin{:});
    
    curve = p.Results.curve;
    m     = p.Results.m;
    plot  = p.Results.plot;
    
    col = distinguishable_colors(10);
    
    % Split the curve into chords
    chords = vecnorm(diff(curve.arc')');
    
    % Calculate the integral of curvature
    totalKappa = 0.5 * chords(1) * curve.kappa(1) + ...
        0.5 * chords(end) * curve.kappa(end);
    
    for ii = 2 : length(curve.arc) - 1
        totalKappa = totalKappa + ...
            curve.kappa(ii) * (chords(ii-1) + chords(ii));
    end
    
    % Now partition the curve into sections.
    % Each section should have total curvature == totalKappa / m
    quantumKappa = totalKappa / m;
    
    t = zeros(1, m);
    arcIndexes = zeros(1, m);
    kappaAcc = 0.5 * chords(1) * curve.kappa(1);
    ii = 2;
    
    
    for jj = 1 : m
        targetKappa = jj * quantumKappa;
        jj
        
        while kappaAcc <= targetKappa
            if ii == length(curve.arc)
                break;
            end
            
            kappaAcc = kappaAcc + ...
                curve.kappa(ii) * (chords(ii-1) + chords(ii))
            
            ii = ii + 1;
        end
        
        t(jj) = curve.l(ii);
        arcIndexes(jj) = ii;
    end
    
    dominantPts = curve.arc(:,arcIndexes);
    dominantPts = [curve.arc(:,1) dominantPts curve.arc(:,end)];
    
    if plot
        figure
        scatter3(curve.arc(1,:), curve.arc(2,:), curve.arc(3,:),'MarkerEdgeColor', col(7,:), 'LineWidth', 2.5);
        axis equal, grid on, hold on
        triad('scale', 1e-3/2, 'linewidth', 2.5)
        scatter3(dominantPts(1,:), dominantPts(2,:), dominantPts(3,:),'filled', 'MarkerEdgeColor', col(9,:), 'LineWidth', 2.5);
%         xlim([min(curve.arc(1,:)) max(curve.arc(1,:))+0.01])
%         ylim([min(curve.arc(2,:)) max(curve.arc(2,:))+0.01])
%         zlim([min(curve.arc(3,:)) max(curve.arc(3,:))+0.01]);
        xlabel('X [m]'), ylabel('Y [m]'), zlabel('Z [m]')
        legend({'Original Curve', 'Dominant Points'});
        view(80, 25);
        set(gca,'FontSize',16);
    end
end

