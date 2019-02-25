classdef Robot
    % Robot This class encapsulates the design parameters of a
    % notched-tube wrist.
    %   Author: Loris Fichera <lfichera@wpi.edu>
    %   Latest revision: 09/18/2018
    %
    % __        __    _     _   ____            _
    % \ \      / / __(_)___| |_|  _ \  ___  ___(_) __ _ _ __
    %  \ \ /\ / / '__| / __| __| | | |/ _ \/ __| |/ _` | '_ \
    %   \ V  V /| |  | \__ \ |_| |_| |  __/\__ \ | (_| | | | |
    %    \_/\_/ |_|  |_|___/\__|____/ \___||___/_|\__, |_| |_|
    %                                             |___/
    %
    properties
        ID        % [mm] tube inner diameter
        OD        % [mm] tube outer diameter
        nCutouts  % [int] total number of cutouts
        cutouts   % struct array - each struct contains the following fields:
        %   [u - spacing between i-1 and i-th notch]
        %   [h - height of i-th cutout             ]
        %   [w - width of i-th cutout              ]
        %   [phi - orientation of i-th cutout      ]
    end
    
    methods
        function self = Robot(ID, OD, nCutouts, cutouts)
            self.ID = ID;
            self.OD = OD;
            self.nCutouts = nCutouts;
            self.cutouts = cutouts;
        end
        
        function [pose, transformations] = fwkine(self, configuration)
            n = self.nCutouts; % total number of cutouts
            ro = self.OD * 10^-3 / 2; % outer radius of tube in [m];
            ri = self.ID * 10^-3 / 2; % inner radius of tube in [m];
            
            % !FIXME later on, we should account for wrists with non-uniform cutouts
            % !FIXME need to implement kinematics for different cut orientations (theta)
            u = self.cutouts(1).u * 10^-3; % Length between cuts in [m]. Defined in Figure 4.
            h = self.cutouts(1).h * 10^-3; % height of the cutouts in [m]
            w = self.cutouts(1).w * 10^-3; % cut depth in [m]. See Figure 4.
            alpha = self.cutouts(1).alpha * pi / 180;       %cut orientation in [rad].
            
            d = w-ro; % intermediate variable. Depth of cut as measured from y = 0. See Figure 4.
            
            % Get the endoscope configuration
            t_displ = configuration(1) * 10^-3;
            t_rot   = configuration(2) * pi / 180;
            t_adv   = configuration(3) * 10^-3;
            
            % Define the H.T. corresponding to rotation
            % about the x-axis and
            % translation in y and z according to
            % constant curvature deformation
            Ts = @(k,s) [1 0 0 0;...
                         0 cos(k*s) -sin(k*s) (cos(k*s)-1)/k;...
                         0 sin(k*s) cos(k*s) sin(k*s)/k;...
                         0 0 0 1];
            
            % Define the H.T. corresponding to translation along z-axis
            Tz = @(a) [1 0 0 0;...
                       0 1 0 0;...
                       0 0 1 a;...
                       0 0 0 1];
            
            % Define the H.T. corresponding to rotation about the z-axis
            Trotz = @(alpha) [cos(alpha) -sin(alpha) 0 0; ...
                              sin(alpha) cos(alpha)  0 0; ...
                              0          0           1 0; ...
                              0          0           0 1];
            
            options = optimoptions('fsolve', 'TolFun', 1e-14, 'Display', 'off'); %set fsolve options.
            
            %% Calculations
            % Calculate ybar (Equation 1 from [Swaney2017]).
            % Use Hunter's integral formula calculation. It yields the same thing as the formula in the paper.
            fy = @(r,theta) (r.*cos(theta));
            fA = @(r,theta) (1);
            phi = @(r) acos(d./r);
            A = integral2(@(r,theta) r.*fA(r,theta), ri, ro, @(c)-phi(c), @(c)phi(c), ...
                'AbsTol', 1e-12, 'RelTol', 1e-12);
            
            ybar = 1/A*integral2(@(r,theta) r.*fy(r,theta), ri, ro, @(c)-phi(c), @(c)phi(c), ...
                'AbsTol',1e-12,'RelTol', 1e-9);
            
            % Calculate the maximum bending for a single cutout
            theta_max = h / (ro + ybar);
            
            % Calculate the tendon displacement for single cutout when
            % this hits hard stop.
            deltal_max = h - (ro - ri) * theta_max;
            
            if t_displ > deltal_max
                disp('Warning. Tendon displacement runs over hard stop.');
                disp(deltal_max * 1000);
            end
            
            % Get kappa of single cutout
            kappa = fsolve(@(k) (-t_displ + h - 2 * (1/k - ri) * sin(k*h/(2*(1+ybar*k)))), ...
                500, options); % original -t
            
            % Get arc length of single cutout
            s = h / ( 1 + ybar * kappa); % original kappa
            
            % Initialize pose and transformation matrices
            pose = zeros(4, n + 2);
            pose(4,:) = ones(1, n + 2);
            T = repmat(eye(4), 1, 1, n + 2);
            
            % Calculate initial advancement and rotation
            T(:,:,2) = Tz(t_adv) * Trotz(t_rot);
            pose(:,2) = T(:,:,2) * [0 0 0 1]';
            
            % Iterate on the cutouts and calculate the transformations at each cutout
            for i = 3 : (n + 2)
                % !FIXME Need to account for orientation of each single
                % cutout
%                 T(:,:,i) = T(:,:,i-1) * ...
%                            Trotz(alpha(i-2)) * Ts(kappa, s) * Tz(u);

                T(:,:,i) = T(:,:,i-1) * ...
                           Trotz(alpha) * Ts(kappa, s) * Tz(u);

                pose(:,i) = T(:,:,i) * [0 0 0 1]';
            end
            
            % Extract the tube pose and return
            pose = pose(1:3,:) .* 1000;
            transformations = T;
            transformations(1:3,4,:) = transformations(1:3,4,:) .*1000;
        end
        
        
        %       function obj = setCutoutOrientations(obj, cutOrientations)
        %           for i = 1 : obj.nCutouts
        %               obj.cutouts(i).phi = cutOrientations(i) * 360;
        %           end
        %       end
        %
        %
        %       function show(self)
        %           n_pts = 50;
        %           faceColor = [0.8 0.8 0.8];
        %           tubeLength = 10; %[mm] - arbitrary - for display purposes only
        %                            % shows only the most distal 10 mm of the tip
        %
        %           hold on
        %
        %           currentZ = tubeLength;
        %
        %           for i = 1 : self.nCutouts
        %               % display uncut section of length 'X'
        %               [Xi,Yi,Zi] = cylinder(self.ID/2, n_pts);
        %               [Xo,Yo,Zo] = cylinder(self.OD/2, n_pts);
        %
        %               % set length of this uncut section
        %               Zi(2,:) = Zi(2,:) - Zi(2,:) + currentZ;
        %               Zi(1,:) = Zi(1,:) - Zi(1,:) + currentZ - self.cutouts(i).x;
        %               Zo(2,:) = Zo(2,:) - Zo(2,:) + currentZ;
        %               Zo(1,:) = Zo(1,:) - Zo(1,:) + currentZ - self.cutouts(i).x;
        %
        %               % plot uncut section
        %               hSurf = surf(Xi,Yi,Zi);
        %               set(hSurf,'FaceColor',faceColor,'FaceAlpha',1);
        %
        %               hSurf = surf(Xo,Yo,Zo);
        %               set(hSurf,'FaceColor',faceColor,'FaceAlpha',1);
        %
        %               % display cut section of length 'H'
        %               [Xi,Yi,Zi] = cylinder(self.ID/2, n_pts);
        %               [Xo,Yo,Zo] = cylinder(self.OD/2, n_pts);
        %
        %               % set length of this cut section
        %               Zi(2,:) = Zi(2,:) - Zi(2,:) + currentZ - self.cutouts(i).x;
        %               Zi(1,:) = Zi(1,:) - Zi(1,:) + currentZ - self.cutouts(i).x - self.cutouts(i).h;
        %               Zo(2,:) = Zo(2,:) - Zo(2,:) + currentZ - self.cutouts(i).x;
        %               Zo(1,:) = Zo(1,:) - Zo(1,:) + currentZ - self.cutouts(i).x - self.cutouts(i).h;
        %
        %               % select only the portion of the tube that has not been cut
        %               sel = Yi(1,:) + self.OD/2 < self.OD - self.cutouts(i).w;
        %               Xi = Xi(:,sel); Yi = Yi(:,sel); Zi = Zi(:,sel);
        %
        %               sel = Yo(1,:) + self.OD/2 < self.OD - self.cutouts(i).w;
        %               Xo = Xo(:,sel); Yo = Yo(:,sel); Zo = Zo(:,sel);
        %
        %               % prepare points for rotation about tube axis
        %               % (cutout rotated by phi)
        %               outerCylinderAbove = [Xo(2,:); Yo(2,:); Zo(2,:)];
        %               outerCylinderBelow = [Xo(1,:); Yo(1,:); Zo(1,:)];
        %               innerCylinderAbove = [Xi(2,:); Yi(2,:); Zi(2,:)];
        %               innerCylinderBelow = [Xi(1,:); Yi(1,:); Zi(1,:)];
        %
        %               % rotate uncut section by phi (design parameter)
        %               phi = self.cutouts(i).phi;
        %               R = [cosd(phi) -sind(phi) 0 0;
        %                    sind(phi) cosd(phi)  0 0;
        %                    0           0        1 0;
        %                    0           0        0 1];
        %
        %               for j = 1 : size(outerCylinderAbove, 2)
        %                   temp = R * [outerCylinderAbove(:,j); 1];
        %                   outerCylinderAbove(:,j) = temp(1:3);
        %
        %                   temp = R * [outerCylinderBelow(:,j); 1];
        %                   outerCylinderBelow(:,j) = temp(1:3);
        %               end
        %
        %
        %               for j = 1 : size(innerCylinderAbove, 2)
        %
        %                   temp = R * [innerCylinderAbove(:,j); 1];
        %                   innerCylinderAbove(:,j) = temp(1:3);
        %
        %                   temp = R * [innerCylinderBelow(:,j); 1];
        %                   innerCylinderBelow(:,j) = temp(1:3);
        %               end
        %
        %               Xo = [outerCylinderBelow(1,:); outerCylinderAbove(1,:)];
        %               Yo = [outerCylinderBelow(2,:); outerCylinderAbove(2,:)];
        %               Zo = [outerCylinderBelow(3,:); outerCylinderAbove(3,:)];
        %
        %               Xi = [innerCylinderBelow(1,:); innerCylinderAbove(1,:)];
        %               Yi = [innerCylinderBelow(2,:); innerCylinderAbove(2,:)];
        %               Zi = [innerCylinderBelow(3,:); innerCylinderAbove(3,:)];
        %
        %
        %               hSurf = surf(Xi,Yi,Zi);
        %               set(hSurf,'FaceColor',faceColor,'FaceAlpha',1);
        %
        %               hSurf = surf(Xo,Yo,Zo);
        %               set(hSurf,'FaceColor',faceColor,'FaceAlpha',1);
        %
        %               currentZ = currentZ - self.cutouts(i).x - self.cutouts(i).h;
        %           end
        %
        %           % display base of the tube
        %           [Xi,Yi,Zi] = cylinder(self.ID/2, n_pts);
        %           [Xo,Yo,Zo] = cylinder(self.OD/2, n_pts);
        %
        %           % set length of this uncut section
        %           Zi(2,:) = Zi(2,:) - Zi(2,:) + currentZ;
        %           Zi(1,:) = 0;
        %           Zo(2,:) = Zo(2,:) - Zo(2,:) + currentZ;
        %           Zo(1,:) = 0;
        %
        %           % plot uncut section
        %           hSurf = surf(Xi,Yi,Zi);
        %           set(hSurf,'FaceColor',faceColor,'FaceAlpha',1);
        %
        %           hSurf = surf(Xo,Yo,Zo);
        %           set(hSurf,'FaceColor',faceColor,'FaceAlpha',1);
        %
        %           axis equal, grid on
        %           %xlabel('X [mm]'), ylabel('Y [mm]'), zlabel('Z [mm]')
        %           view(40.8, 24.4);
        %       end
        %    end
    end
end
