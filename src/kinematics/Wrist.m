classdef Wrist < Robot
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
        
        ybar
        theta_max
        deltal_max
    end
    
    methods
        function self = Wrist(ID, OD, nCutouts, cutouts)
            % Invoke the constructor of the parent class
            nLinks = nCutouts * 2 + 1;
            self@Robot(nLinks);
            
            % Copy geometric design parameters in local attributes
            self.ID = ID;
            self.OD = OD;
            self.nCutouts = nCutouts;
            self.cutouts = cutouts;
            
            % Pre-calculate some variables to make the forward kinematics run faster
            ro = self.OD * 10^-3 / 2; % outer radius of tube in [m];
            ri = self.ID * 10^-3 / 2; % inner radius of tube in [m];
            
            h = self.cutouts.h .* 10^-3; % Height of the cutouts in [m]
            w = self.cutouts.w .* 10^-3; % Cut depth in [m]. See Figure 4 again.            
            d = w-ro; % intermediate variable. Depth of cut as measured from y = 0. See Figure 4.
            
            
            for ii = 1 : nCutouts
                phio = 2 * acos(d(ii) / ro);
                phii = 2 * acos(d(ii) / ri);
                
                ybaro = (4 * ro * (sin(0.5 * phio)) ^ 3)/ (3 * (phio - sin(phio)));
                ybari = (4 * ri * (sin(0.5 * phii)) ^ 3)/ (3 * (phio - sin(phii)));
                
                Ao = ( (ro ^ 2) * ( phio - sin(phio))) / 2;
                Ai = ( (ri ^ 2) * ( phii - sin(phii))) / 2;
                
                self.ybar(ii) = (ybaro * Ao - ybari * Ai) / (Ao - Ai);
                
                % Calculate the maximum bending for this cutout
                self.theta_max(ii) = h(ii) / (ro + self.ybar(ii));
                
                % Calculate the tendon displacement for this cutout when
                % this hits the hard stop.
                self.deltal_max(ii) = h(ii) - (ro - ri) * self.theta_max(ii);                
            end
        end
        
        function c = jointvar2arcparams(self, q)
            % == Get the endoscope joint variables q
            t_displ = q(1) * 10^-3; % tendon displacement [m]
            t_rot   = q(2);         % tube rotation [m]
            t_adv   = q(3) * 10^-3; % 
            
            % == Read the geometric design parameters
            ro = self.OD * 10^-3 / 2; % outer radius of tube in [m];
            ri = self.ID * 10^-3 / 2; % inner radius of tube in [m];
            
            h = self.cutouts.h .* 10^-3; % Height of the cutouts in [m]
            w = self.cutouts.w .* 10^-3; % Cut depth in [m]. See Figure 4 again.            
            d = w-ro; % intermediate variable. Depth of cut as measured from y = 0. See Figure 4.
            
            % == Perform a robot-specific mapping from the joint variables q
            % to the vector of arc parameters c
            c = zeros(1, 3 * self.nLinks);
   
            % initialize a counter to iterate on the notches
            kk = 1;
   
            for jj = 0 : self.nLinks - 1
                % Even-numbered links are notches
                % Odd-numbered links are straight sections
                
                if mod(jj+1,2) == 1
                    % For a straight section, the curvature is always zero
                    kjj = 0;
                    
                    if jj+1 == 1
                        % For the first staight section only, the length is
                        % given by the robot advancement, and the rotation
                        % is given by the tube rotation
                        sjj = t_adv;
                        thetajj = t_rot;
                    else
                        % For all successive uncut sections, there is no
                        % rotation, and the advancement is given by the
                        % length of the uncut section itelf
                        sjj = self.cutouts.u(kk-1) .* 10^-3; % Uncut section [m]
                        thetajj = 0;
                    end
                 
                else
                    % For a curved section (i.e. a notch), calculate the
                    % arc parameters using the relations in [Swaney2017]
                    kjj = (t_displ) / (h(kk) * (ri + self.ybar(kk)) - t_displ * self.ybar(kk));
                    sjj = h(kk) / ( 1 + self.ybar(kk) * kjj); % original kappa
                    thetajj = 0;
                    
                    % Save the curvature and arc length of each notch
                    % !FIXME this may be different for each cutout
                    self.curvature = kjj;
                    self.arcLength = sjj;
                   
                    % move to the next cutout
                    kk = kk + 1;
                end
                
                c(jj*3+1:jj*3+3) = [kjj sjj thetajj];
            end
        end
        
        function fwkine(self, q, baseTransform)
            % map joint variables to arc parameters
            c = self.jointvar2arcparams(q);

            % Calculate the robot-independent mapping and return the result
            [P,T] = fwkine@Robot(self, c, baseTransform);
            
            % Save pose and transformations in local attributes
            self.pose = P(1:3,:) .* 1000;
            self.transformations = T;
            self.transformations(1:3,4,:) = self.transformations(1:3,4,:) .*1000;
        end
        
        
        function J = jacob0(self, q)
            % Read the joint variables
            t_displ = q(1) * 10^-3; % tendon displacement [m]
            t_rot   = q(2);         % tube rotation [m]
            t_adv   = q(3) * 10^-3; % 
            
            % Read the geometric design parameters
            ro = self.OD * 10^-3 / 2; % outer radius of tube in [m];
            ri = self.ID * 10^-3 / 2; % inner radius of tube in [m];
            
            h = self.cutouts.h .* 10^-3; % Height of the cutouts in [m]
            
            % Map joint variables to arc parameters
            c = self.jointvar2arcparams(q);
            
            % Calculate the robot-independent Jacobian
            J = jacob0@Robot(self, c);
            
            % Expand the time derivatives of the arc parameters using the
            % chain rule
            cdot = zeros(length(c), 1);
            cdotmatrix = zeros(3*self.nLinks, 3);
            
            % initialize a counter to iterate on the notches
            kk = 1;
            
            for jj = 0 : self.nLinks - 1
                % Even-numbered links are notches
                % Odd-numbered links are straight sections
                
                if mod(jj+1,2) == 1 % for a straight section
                    kdotjj = 0;
                    
                    if jj+1 == 1    
                        % For the first staight section only, the rate of
                        % change of advancement and rotation are directly
                        % controlled through manipulation of the joint
                        % variables
                        sdotjj     = 1;
                        thetadotjj = 1;
                    else
                        % For all successive uncut sections, there is no
                        % translational nor rotational velocity;
                        sdotjj     = 0;
                        thetadotjj = 0;
                    end
                else % for a notched section
                    d = (h(kk) * (ri + self.ybar(kk)) - t_displ * self.ybar(kk));
                    kdotjj = 1/d + self.ybar(kk) *  t_displ / d^2;
                    sdotjj = -(h(kk) * self.ybar(kk) / d + self.ybar(kk)^2 * t_displ / d^2) * ...
                        1 / (1 + (t_displ * self.ybar(kk)) / d)^2;
                    thetadotjj = 0;
                    
                    % move on to the next cutout
                    kk = kk + 1;
                end
                
                cdot(jj*3+1:jj*3+3) = [kdotjj sdotjj thetadotjj];
                cdotmatrix(jj*3+1,1) = kdotjj;
                
                if jj+1 == 1
                    cdotmatrix(jj*3+2,3) = sdotjj;
                else
                    cdotmatrix(jj*3+2,1) = sdotjj;
                end
                
                cdotmatrix(jj*3+3,2) = thetadotjj;
            end
            
            J = J*cdotmatrix;
        end
        %         function invkine(self, p, q)
        %             qCurrent = q;
%             cCurrent = c;
%             
%             while true
%                 % calculate the current location
%                 cCurrent = self.jointvar2arcparams(qCurrent);
%                 self.fwkine(qCurrent, eye(4));                
%                 pCurrent = self.pose(1:3,end);
% 
%                 
%                 % calculate the difference between current and target location
%                 err = norm(pTarget - pCurrent);
%                 
%                 % if the difference < epsilon, return
%                 if err < 1e-1, break; end
%                 
%                 % else, update the "joint" variables using the inverse of the
%                 % jacobian
%                 J = r.jacob(c);
%                 Jp = J(1:3,:) - skew(pCurrent) * J(4:6,:);
%                 Jpinv = pinv(Jp);
%                 
%                 K = diag([1 1 1 1 1 1 0 1 1]);
%                 deltaQ = K * Jpinv * (pTarget - pCurrent);
%                 c = c + deltaQ;
%             end
%             
%             
%         end
%         
        
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
                if mod(ii,2) == 1 || kappa == 0% straight sections
                    
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
            [X,Y,Z] = gencyl(robotBackbone, radiusVec, 2, 20);
            
            robotModel.backbone = robotBackbone;
            robotModel.surface.X = X;
            robotModel.surface.Y = Y;
            robotModel.surface.Z = Z;
            self.robotModel = robotModel;
        end
    end
end
