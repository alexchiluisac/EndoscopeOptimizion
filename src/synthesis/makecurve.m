function curve = makecurve(varargin)
    %% MAKECURVE generates a curve based on desired curvature and torsion profiles

    % Input handling
    defaultArcLength = 2e-3;
    variableK = 100;
    defaultK         = @(s,arcLength) variableK .* ones(1, length(s));
    defaultTau       = @(s,arcLength) 0 * s/arcLength;
    defaultTransform = eye(4);
    defaultRotation  = 0;
    defaultPlot      = false;
    
    p = inputParser;
    addOptional(p, 'arcLength', defaultArcLength);
    addOptional(p, 'k', defaultK);
    addOptional(p, 'tau', defaultTau);
    addOptional(p, 'transform', defaultTransform);
    addOptional(p, 'rotation', defaultRotation);
    addParameter(p, 'plot', defaultPlot);
    parse(p, varargin{:});
    
    arcLength = p.Results.arcLength;
    k         = p.Results.k;
    tau       = p.Results.tau;
    transform = p.Results.transform;
    rotation  = p.Results.rotation;
    plot      = p.Results.plot;
    
    col = distinguishable_colors(10);
    
    % Numerically solve the Frenet-Serret equations
    % t -> x(1) x(2) x(3)
    % n -> x(4) x(5) x(6)
    % b -> x(7) x(8) x(9)
    f = @(s,x) [k(s,arcLength)*x(4);  k(s,arcLength)*x(5); k(s,arcLength)*x(6);
        -k(s,arcLength)*x(1) + tau(s,arcLength)*x(7); -k(s,arcLength)*x(2) + tau(s,arcLength)*x(8); -k(s,arcLength)*x(3) + tau(s,arcLength)*x(9);
        -tau(s,arcLength)*x(4); -tau(s,arcLength)*x(5); -tau(s,arcLength)*x(6)];
    
    [l,y] = ode45(f, [0 arcLength], [0 0 1 1 0 0 0 1 0]);
    
    t = [y(:,1) y(:,2) y(:,3)]';
    n = [y(:,4) y(:,5) y(:,6)]';
    b = [y(:,7) y(:,8) y(:,9)]';
    
    % Generate the arc points by integration of the t vector along s
    initialPosition = transform(1:3, 4);
    arc = initialPosition;
    
    scalefactor = 10e2;
    
    Trotz = [cos(rotation) -sin(rotation) 0; ...
              sin(rotation) cos(rotation)  0; ...
              0          0           1];

    for ii = 2 : size(l, 1)
        arc(:,ii) = initialPosition + ( (transform(1:3, 1:3) * Trotz * [trapz(l(1:ii), t(1,1:ii));
            trapz(l(1:ii), t(2,1:ii));
            trapz(l(1:ii), t(3,1:ii))]) .* scalefactor);
    end
    
    curve.arc   = arc ;
    curve.l     = l;
    curve.kappa = k(l,arcLength);
    curve.tau   = tau(l,arcLength);
    curve.t     = t;
    curve.n     = n;
    curve.b     = b;
    
    if plot
        % Plot the resulting line
        figure
        scatter3(arc(1,:), arc(2,:), arc(3,:),'MarkerEdgeColor', col(5,:), 'LineWidth', 2.5);
        hold on, axis equal, grid on
        %xlim([-2e-3 2e-3]), ylim([-2e-3 2e-3]), zlim([0 5e-3]);
        xlabel('X [m]'), ylabel('Y [m]'), zlabel('Z [m]'),
        view(0.26, 20.5)
        set(gca,'FontSize',16);
        title('Target Curve');
        
        h = triad('scale', 1e-3/2, 'linewidth', 2.5);
        
%         % Make an animation showing the Frenet-Serret frames
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