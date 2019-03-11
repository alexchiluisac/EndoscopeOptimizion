classdef Wrist % !FIXME this should be a subclass of Robot
    % WRIST This class encapsulates the design parameters of a
    % notched-tube wrist.
    %
    %   Author: Loris Fichera <lfichera@wpi.edu>
    %
    %   Latest revision: 03/01/2019
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
        function self = Wrist(ID, OD, nCutouts, cutouts)
            self.ID = ID;
            self.OD = OD;
            self.nCutouts = nCutouts;
            self.cutouts = cutouts;
        end
        
        function [pose, transformations, curvature, arcLength] = fwkine(self, configuration)
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
            t_rot   = configuration(2);
            t_adv   = configuration(3) * 10^-3;
            
            % Define the H.T. corresponding to rotation
            % about the y-axis and
            % translation in x and z according to
            % constant curvature deformation
            Ts = @(k,s) [cos(k*s) 0  sin(k*s) (1 - cos(k*s))/k;
                0        1  0        0;
                -sin(k*s) 0 cos(k*s) sin(k*s)/k;
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
            pose = zeros(4, 2*n + 2);
            pose(4,:) = ones(1, 2*n + 2);
            T = repmat(eye(4), 1, 1, 2*n + 2);
            
            % Calculate initial advancement and rotation
            T(:,:,2) = Tz(t_adv) * Trotz(t_rot);
            pose(:,2) = T(:,:,2) * [0 0 0 1]';
            
            % Iterate on the cutouts and calculate the transformations at each cutout
            for i = 3 : 2 : (2*n + 2)
                % !FIXME Need to account for orientation of each single
                % cutout
                
                T(:,:,i) = T(:,:,i-1) * ...
                    Trotz(alpha) * Ts(kappa, s);% * Tz(u);
                T(:,:,i+1) = T(:,:,i) * Tz(u);
                
                pose(:,i) = T(:,:,i) * [0 0 0 1]';
                pose(:,i+1) = T(:,:,i+1) * [0 0 0 1]';
            end
            
            % Extract the tube pose and return
            pose = pose(1:3,:) .* 1000;
            transformations = T;
            transformations(1:3,4,:) = transformations(1:3,4,:) .*1000;
            
            % Return the curvature and arc length of each cut section
            curvature = kappa;
            arcLength = s;
        end
        
        
        function robotModel = makePhysicalModel(self, configuration, baseTransform)
            ptsPerMm = 10;
            
            [P,T,kappa,s] = self.fwkine(configuration);
            kappa = kappa / 1000;
            radius = 1 / kappa;
            s = s * 1000;
            
            robotBackbone = zeros(3,1);
            
            for ii = 1 : size(P, 2) - 1
                if mod(ii,2) == 1 % straight sections
                    
                    % generate points along a straight line
                    distance = norm(P(:,ii+1) - P(:,ii));
                    nPts = round(distance * ptsPerMm);
                    
                    X = linspace(P(1,ii),P(1,ii+1), nPts);
                    Y = linspace(P(2,ii),P(2,ii+1), nPts);
                    Z = linspace(P(3,ii),P(3,ii+1), nPts);
                    
                    robotBackbone = [robotBackbone [X;Y;Z]];  
                
                else % cutouts: curved sections
                    
                    % generate points along an arc of constant curvature
                    % and of length s
                    
                    % !FIXME ensure ptspermm is met
                    theta = 0 : s*kappa/10 : s*kappa;
                   
                    pts = radius .* [(1-cos(theta));
                                     zeros(1, length(theta));
                                     sin(theta);
                                     ones(1, length(theta)) / radius];
                                  
                    robotBackbone = [robotBackbone ...
                        applytransform(pts(1:3,:),T(:,:,ii))]; 
                end
            end 
            
            robotBackbone = applytransform(robotBackbone, baseTransform);
            
            radiusVec = self.OD/2*ones(1,size(robotBackbone,2));
            [X,Y,Z] = gencyl(robotBackbone, radiusVec);
            
            robotModel.backbone = robotBackbone;
            robotModel.surface.X = X;
            robotModel.surface.Y = Y;
            robotModel.surface.Z = Z;
        end
    end
end
