function curve = makecurve(varargin)
    %% MAKECURVE generates a curve based on desired curvature and torsion profiles

    % Input handling
    defaultArcLength = 10e-3;
    defaultK         = @increasing;
    defaultTau       = @(s, arcLength) 0 * s/arcLength;
    defaultPlot      = false;
    defaultBaseTransform = eye(4);
    
    p = inputParser;
    addOptional(p, 'arcLength', defaultArcLength);
    addOptional(p, 'k', defaultK);
    addOptional(p, 'tau', defaultTau);
    addOptional(p, 'baseTransform', defaultBaseTransform);
    addParameter(p, 'plot', defaultPlot);
    parse(p, varargin{:});
    
    arcLength = p.Results.arcLength;
    k         = p.Results.k;
    tau       = p.Results.tau;
    baseTransform = p.Results.baseTransform;
    plot      = p.Results.plot;
        
    % Numerically solve the Frenet-Serret equations
    % t -> x(1) x(2) x(3)
    % n -> x(4) x(5) x(6)
    % b -> x(7) x(8) x(9)
    f = @(s,x) [k(s,arcLength)*x(4); 
                k(s,arcLength)*x(5);
                k(s,arcLength)*x(6); 
               -k(s,arcLength)*x(1) + tau(s,arcLength)*x(7);
               -k(s,arcLength)*x(2) + tau(s,arcLength)*x(8);
               -k(s,arcLength)*x(3) + tau(s,arcLength)*x(9);
               -tau(s,arcLength)*x(4);
               -tau(s,arcLength)*x(5);
               -tau(s,arcLength)*x(6)];
    
    [l,y] = ode45(f, 0:1e-4:arcLength, [0 0 1 1 0 0 0 1 0]);
    
    t = [y(:,1) y(:,2) y(:,3)]';
    n = [y(:,4) y(:,5) y(:,6)]';
    b = [y(:,7) y(:,8) y(:,9)]';
    
    % Generate the arc points by integration of the t vector along s
    arc = zeros(3, size(l, 1));
    
    for ii = 2 : size(l, 1)
        arc(:,ii) = [trapz(l(1:ii), t(1,1:ii));
            trapz(l(1:ii), t(2,1:ii));
            trapz(l(1:ii), t(3,1:ii))];
    end
    
    curve.arc   = applytransform(arc, baseTransform);
    curve.l     = l;
    curve.t     = applytransform(t, baseTransform);
    curve.n     = applytransform(n, baseTransform);
    curve.b     = applytransform(b, baseTransform);
    
    curve.kappa = zeros(1, length(l));
    curve.tau   = zeros(1, length(l));
    
    for ii = 1 : length(l)
        curve.kappa(ii) = k(l(ii),arcLength);
        curve.tau(ii)   = tau(l(ii),arcLength);
    end
    
    if plot
        % Plot the resulting line
        figure
        plot3(arc(1,:), arc(2,:), arc(3,:), 'LineWidth',3,'Color',(1/256)*[255 128 0]);
        hold on, axis equal, grid on
        xlabel('X [m]'), ylabel('Y [m]'), zlabel('Z [m]'),
        view(0.26, 20.5)
        set(gca,'FontSize',16);
        title('Target Curve');
        
        triad('scale', 1e-3/2, 'linewidth', 2.5);
        h = triad('scale', 1e-3/2, 'linewidth', 2.5);
        
        % Make an animation showing the Frenet-Serret frames
%         for ii = 2 : size(l, 1)
%             rot = [n(:,ii) b(:,ii) t(:,ii)];
%             transl = arc(:,ii);
%             T = [rot transl; 0 0 0 1];
%             
%             h.Matrix = T;
%             pause(0.1);
%             drawnow
%         end
    end
end

function v = uniform(s, arcLength)
v = pi/(2 * arcLength);
end

function v = increasing(s, arcLength)
v = 300 * s / arcLength;
end