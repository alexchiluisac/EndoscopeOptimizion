classdef Wrist < handle % !FIXME this should be a subclass of Robot
    % WRIST This class encapsulates the design parameters of a
    % notched-tube wrist.
    %
    %   Author: Loris Fichera <lfichera@wpi.edu>
    %
    %   Edited by Floris van Rossum to make optimization changes.
    %   Latest revision: 06/05/2019
    properties
        ID        % [mm] tube inner diameter
        OD        % [mm] tube outer diameter
        nCutouts  % [int] total number of cutouts
        cutouts   % struct array - each struct contains the following fields:
        %   [u - spacing between i-1 and i-th notch]
        %   [h - height of i-th cutout             ]
        %   [w - width of i-th cutout              ]
        %   [phi - orientation of i-th cutout      ]
        
        % Forward Kinematics Parameters
        pose                % The position and orientation
        transformations     % Transformation matrix
        curvature           % Curvature of the wrist links
        arcLength           % Arc length of the wrist links
        robotModel          % A model of the robot
    end
    
    methods
        function self = Wrist(ID, OD, nCutouts, cutouts)
            self.ID = ID;
            self.OD = OD;
            self.nCutouts = nCutouts;
            self.cutouts = cutouts;
        end
        
        function fwkine(self, configuration, baseTransform)
            % Get the endoscope configuration
            t_displ = configuration(1) * 10^-3;
            t_rot   = configuration(2);
            t_adv   = configuration(3) * 10^-3;
            
            % Get the kinematic parameters
            n = self.nCutouts; % total number of cutouts
            ro = self.OD * 10^-3 / 2; % outer radius of tube in [m];
            ri = self.ID * 10^-3 / 2; % inner radius of tube in [m];
            
            % !FIXME later on, we should account for wrists with non-uniform cutouts
            % !FIXME need to implement kinematics for different cut orientations (theta)
            u = self.cutouts.u .* 10^-3; % Length between cuts in [m]. Defined in Figure 4 of Swaney 2017.
            h = self.cutouts.h .* 10^-3; % Height of the cutouts in [m]
            w = self.cutouts.w .* 10^-3; % Cut depth in [m]. See Figure 4 again.
            alpha = self.cutouts.alpha ; % Change in cut orientation in [rad].
            
            baseTransform(1:3,end) = baseTransform(1:3,end)./1000; % converting from [mm] to [m]
            
            d = w-ro; % intermediate variable. Depth of cut as measured from y = 0. See Figure 4.
            

            
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
            
            %% Calculations
            % Calculate ybar (Equation 1 from [Swaney2017]).
            % Use Hunter's integral formula calculation. It yields the same thing as the formula in the paper.
            fy = @(r,theta) (r.*cos(theta));
            fA = @(r,theta) (1);
            %phi = @(r) acos(d./r);
            
            A =     zeros(1, n);
            ybar =  zeros(1, n);
            kappa = zeros(1, n);
            s =     zeros(1, n);
            
            
                        
            for ii = 1 : n
                phio = 2 * acos(d(ii) / ro);
                phii = 2 * acos(d(ii) / ri);
                
                ybaro = (4 * ro * (sin(0.5 * phio)) ^ 3)/ (3 * (phio - sin(phio)));
                ybari = (4 * ri * (sin(0.5 * phii)) ^ 3)/ (3 * (phii - sin(phii)));
                
                Ao = ( (ro ^ 2) * ( phio - sin(phio))) / 2;
                Ai = ( (ri ^ 2) * ( phii - sin(phii))) / 2;
                
                ybar(ii) = (ybaro * Ao - ybari * Ai) / (Ao - Ai);
%                 A(ii) = integral2(@(r,theta) r.*fA(r,theta), ri, ro, ...
%                                   @(c) -acos(d(ii)./c), @(c) acos(d(ii)./c), ...
%                                   'AbsTol', 1e-12, 'RelTol', 1e-12);
%                 
%                 ybar(ii) = 1 / A(ii) * integral2(@(r,theta) r.*fy(r,theta), ri, ro, ...
%                                              @(c) -acos(d(ii)./c), @(c) acos(d(ii)./c), ...
%                                              'AbsTol',1e-12,'RelTol', 1e-9);
                
                % Calculate the maximum bending for this cutout
                theta_max(ii) = h(ii) / (ro + ybar(ii));
                
                % Calculate the tendon displacement for this cutout when
                % this hits the hard stop.
                deltal_max(ii) = h(ii) - (ro - ri) * theta_max(ii);
                
                if t_displ > deltal_max(ii)
                    disp('Warning. Tendon displacement runs over hard stop.');
                    disp(deltal_max * 1000);
                    
                    % Throw an error
                    % Used to interrupt interface drawing
                    msgID = 'WRIST:TENDON_ERR';
                    msg = 'Tendon displacement runs over hard stop.';
                    tendonStop = MException(msgID, msg);
                    throw(tendonStop);
                end
                
                % Get kappa of single cutout
                
                kappa(ii) = (t_displ) / (h(ii) * (ri + ybar(ii)) - t_displ * ybar(ii));
                % kappa(ii) = fsolve(@(k) (-t_displ + h(ii) - 2 * (1/k - ri) * sin(k*h(ii)/(2*(1+ybar(ii)*k)))), ...
                    % 500, options); % original -t
                
                % Get arc length of single cutout
                s(ii) = h(ii) / ( 1 + ybar(ii) * kappa(ii)); % original kappa
                
            end
            
            % Initialize pose and transformation matrices
            pose = zeros(4, 2*n + 2);
            pose(1:3,1) = baseTransform(1:3,end);
            pose(4,:) = ones(1, 2*n + 2);
            T = repmat(eye(4), 1, 1, 2*n + 2);
            
            % Calculate initial advancement and rotation
            T(:,:,1) = baseTransform;
            T(:,:,2) = baseTransform * Tz(t_adv) * Trotz(t_rot);
            pose(:,2) = T(:,:,2) * [0 0 0 1]';
            
            jj = 1; % pointer to current cutout
            
            % Iterate on the cutouts and calculate the transformations at each cutout
            for i = 3 : 2 : (2*n + 2)
%                 T(:,:,i) = T(:,:,i-1) * ...
%                     Trotz(alpha(jj)) * Ts(kappa(jj), s(jj));% * Tz(u);
%                 T(:,:,i+1) = T(:,:,i) * Tz(u(jj));

                T(:,:,i) = T(:,:,i-1) * Ts(kappa(jj), s(jj));
                
                if jj < n
                    T(:,:,i+1) = T(:,:,i) * Tz(u(jj)) * Trotz(alpha(jj+1));
                else
                    T(:,:,i+1) = T(:,:,i) * Tz(u(jj));
                end

                pose(:,i) = T(:,:,i) * [0 0 0 1]';
                pose(:,i+1) = T(:,:,i+1) * [0 0 0 1]';
                
                jj = jj + 1; % move pointer to next cutout
            end
            
            % Extract the tube pose and return
            self.pose = pose(1:3,:) .* 1000;
            self.transformations = T;
            self.transformations(1:3,4,:) = self.transformations(1:3,4,:) .*1000;
            % Return the curvature and arc length of each cut section
            self.curvature = kappa;
            self.arcLength = s;
        end
        
        
        function robotModel = makePhysicalModel(self)
            ptsPerMm = 5;
            
            P = self.pose;
            T = self.transformations;
            kappa = self.curvature;
            s = self.arcLength;
            %!FIXME kappa and s are different for each cutout - need to change
            %the code below to account for this
            kappa = kappa(1);
            s = s(1);
            
            kappa = kappa / 1000;
            radius = 1 / kappa;
            s = s * 1000;
            
            robotBackbone = P(:,1);
            %robotModel.surface.X = zeros(1, 21);
            %robotModel.surface.Y = zeros(1, 21);
            %robotModel.surface.Z = zeros(1, 21);
            
            for ii = 1 : size(P, 2) - 1
                if mod(ii,2) == 1 % straight sections
                    
                    % generate points along a straight line
                    distance = norm(P(:,ii+1) - P(:,ii));
                    nPts = round(distance * ptsPerMm);
                    
                    X = linspace(P(1,ii),P(1,ii+1), nPts);
                    Y = linspace(P(2,ii),P(2,ii+1), nPts);
                    Z = linspace(P(3,ii),P(3,ii+1), nPts);
                    
                    robotBackbone = [robotBackbone [X;Y;Z]];
                    
                    %[Xcyl,Ycyl,Zcyl] = gencyl([X;Y;Z], self.OD/2*ones(length(X), 1));
                    %robotModel.surface.X = [robotModel.surface.X; Xcyl];
                    %robotModel.surface.Y = [robotModel.surface.Y; Ycyl];
                    %robotModel.surface.Z = [robotModel.surface.Z; Zcyl];
                
                else % cutouts: curved sections
                    
                    % generate points along an arc of constant curvature
                    % and of length s
                    
                    % !FIXME ensure ptspermm is met
                    theta = 0 : s*kappa/10 : s*kappa;
                   
                    pts = radius .* [(1-cos(theta));
                                     zeros(1, length(theta));
                                     sin(theta);
                                     ones(1, length(theta)) / radius];
                    
                    %[Xcyl,Ycyl,Zcyl] = gencyl(pts(1:3,:), self.OD/2*ones(length(pts),1));
     
                    robotBackbone = [robotBackbone ...
                        applytransform(pts(1:3,:), T(:,:,ii))]; 
                    
                    %[Xcyl,Ycyl,Zcyl] = gencyl([X;Y;Z], self.OD/2*ones(length(X),1));
                    %robotModel.surface.X = [robotModel.surface.X; Xcyl];
                    %robotModel.surface.Y = [robotModel.surface.Y; Ycyl];
                    %robotModel.surface.Z = [robotModel.surface.Z; Zcyl];
                end
            end 
            
            %robotBackbone = applytransform(robotBackbone, baseTransform);
            
            radiusVec = self.OD/2*ones(1,size(robotBackbone,2));
            [X,Y,Z] = gencyl(robotBackbone, radiusVec, 2, 7);
            
            robotModel.backbone = robotBackbone;
            robotModel.surface.X = X;
            robotModel.surface.Y = Y;
            robotModel.surface.Z = Z;
            self.robotModel = robotModel;
        end
    end
end
