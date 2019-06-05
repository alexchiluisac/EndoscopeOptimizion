function drawNotched(app)
%DRAWNOTCHED Draw a notched tube with notches and arrows to the notches
%   Created as a visual explanation of the currently designed tube in the
%   design tab.
ax = app.NotchedAxes;
cla(ax); % clear the axes

cuts = app.wrist.cutouts; % Cuts in the wrist
od = app.wrist.OD; % Outer dimension of the wrist
counter = 1;
previousZ = 0;

% Iterate through all the cuts
while counter <= app.wrist.nCutouts
    %% Draw the tube
    [X,Y,Z] = cylinder(od);
    Z(1,:) = previousZ;
    Z(2,:) = cuts.u(counter) + previousZ;
    mesh(ax, X, Y, Z ,'facecolor', '#2f343d', 'edgecolor', '#6d7077')
    
    %% Draw the cut-out semi-circle
    if counter ~= app.wrist.nCutouts
        [theta, rho] = cart2pol(X, Y);
        alpha = cuts.alpha(counter);
        
        alpha = mod(alpha, pi);
        
        if alpha < 0
            alpha = alpha + pi;
        end
        
        minAlpha = alpha;
        
        maxAlpha = minAlpha - pi;
        
        theta( theta < minAlpha & theta > maxAlpha) = minAlpha;
        
        [X, Y] = pol2cart(theta, rho);
        
        Z(1,:) = cuts.u(counter) + previousZ;
        Z(2,:) = cuts.h(counter) + cuts.u(counter) + previousZ;
        surf(ax, X, Y, Z, 'facecolor', '#2f343d', 'edgecolor', '#6d7077');
        previousZ = cuts.h(counter) + cuts.u(counter) + previousZ;
        
    end
    counter = counter + 1;
end
end

