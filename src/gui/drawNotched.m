function drawNotched(app)
%DRAWNOTCHED Draw a notched tube with notches and arrows to the notches
%   Created as a visual explanation of the currently designed tube in the
%   design tab.
currentlySelected = str2double(app.NotchID.Value);
ax = app.NotchedAxes;
cla(ax); % clear the axes

cuts = app.wrist.cutouts; % Cuts in the wrist
od = app.wrist.OD; % Outer dimension of the wrist
counter = 1;
previousZ = 0;

%% Draw cannulae
baseZ = 2;
[X,Y,Z] = cylinder(od);
Z(1,:) = 0;
Z(2,:) = baseZ;
surf(ax, X, Y, Z ,'facecolor', '#232528', 'edgecolor', '#585d68','LineWidth', 0.003)

previousZ = baseZ;

% Iterate through all the cuts
while counter <= app.wrist.nCutouts
    %% Draw the cut-out semi-circle
    [X,Y,Z] = cylinder(od);
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
    Z(1,:) = previousZ;
    Z(2,:) = cuts.h(counter) + previousZ;
    
    % Draw the surface of the cut
    if currentlySelected == counter
        surf(ax, X, Y, Z, 'facecolor', '#721a1a', 'edgecolor', '#585d68');
    else
        surf(ax, X, Y, Z, 'facecolor', '#5cb5db', 'edgecolor', '#585d68');
    end
    %% Draw the tube
    [X,Y,Z] = cylinder(od);
    Z(1,:) = previousZ + cuts.h(counter);
    Z(2,:) = cuts.u(counter) + cuts.h(counter) + previousZ;
    surf(ax, X, Y, Z ,'facecolor', '#5cb5db', 'edgecolor', '#585d68','LineWidth', 0.003)
    
    % Update the previous Z for the next cut
    previousZ = cuts.h(counter) + cuts.u(counter) + previousZ;
    
    counter = counter + 1;
end
end

