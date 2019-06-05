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
    surf(ax, X, Y, Z ,'facecolor', '#5cb5db', 'edgecolor', '#585d68','LineWidth', 0.003)
    
    %% Draw the cut-out semi-circle
    if counter ~= app.wrist.nCutouts
        % Only cut out n-1
        [theta, rho] = cart2pol(X, Y); % Convert to cartesian coordinates
        
        % Get the alpha of the cut
        alpha = cuts.alpha(counter);
        alpha = mod(alpha, pi);
        
        if alpha < 0
            alpha = alpha + pi;
        end
        
        % Calculate along what angle to cut
        minAlpha = alpha;
        
        maxAlpha = minAlpha - pi;
        
        theta( theta < minAlpha & theta > maxAlpha) = minAlpha;
        
        % Convert back to cartesian coordinates to allow for plotting
        [X, Y] = pol2cart(theta, rho);
        
        % Calculate the Z-height of the cut
        Z(1,:) = cuts.u(counter) + previousZ;
        Z(2,:) = cuts.h(counter) + cuts.u(counter) + previousZ;
        
        % Draw the surface of the cut
        surf(ax, X, Y, Z, 'facecolor', '#5cb5db', 'edgecolor', '#585d68');
        
        % Update the previous Z for the next cut
        previousZ = cuts.h(counter) + cuts.u(counter) + previousZ;
        
    end
    counter = counter + 1;
end
end

