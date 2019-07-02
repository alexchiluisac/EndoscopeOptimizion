classdef Robot < handle
    % Robot This class implements the robot-independent kinematics of a continuum robot.
    %   Author: Loris Fichera <lfichera@wpi.edu>
    %   Latest revision: 07/02/2019

    properties
        nLinks % number of links
    end
    
    methods
        function self = Robot(nLinks)
            self.nLinks = nLinks;
        end
        
        function [P, T] = fwkine(self, c, baseTransform)
            % c is the configuration vector, i.e. the vector that contains
            % the arc parameters for each link in the following order:
            % curvature, arc length, base rotation (k, s, theta)
            %
            % baseTransform is an optional parameter that defines a
            % transformation to be applied to the base of the tube -
            % useful, for instance, to register the workspace of the robot
            % with another space
            
            % == Iteratively calculate the forward kinematics ==
            % 1. Initialize pose and transformation matrices
            T = repmat(eye(4), 1, 1, self.nLinks + 1);
            T(:,:,1) = baseTransform;
            
            P = zeros(4, self.nLinks + 1);
            P(1:3,1) = baseTransform(1:3,end);
            P(4,:) = ones(1, self.nLinks + 1);
            
            % 2. Using the product of exponentials formula, generate the
            % transformations for each link and propagate the
            % transformations
            for jj = 0 : self.nLinks - 1
                cjj = c(jj*3+1:jj*3+3);                
                T(:,:,jj+2) = T(:,:,jj+1) * arckinematics(cjj);
                P(1:3,jj+2) = T(1:3,end,jj+2);
            end
        end
    end
end
