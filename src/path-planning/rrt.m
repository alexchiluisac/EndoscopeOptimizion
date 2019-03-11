function [qListNormalized,qList,pList,aList] = rrt(robot, qbounds, anatomyModel)
% RRT implements the basic Rapidly-Exploring Random Trees algorithm for a
% generic continuum robot
%
% Author: L. Fichera <lfichera@wpi.edu>
%
% Last revision: 3/6/2019

    if nargin < 3
        collisionDetection = false;
    else
        collisionDetection = true;
    end
    
    nPoints = 1000;
    
    % algorithm parameters
    deltaQ = [0.05 0.05 0.05]; % step
    maxDispl = qbounds(1);
    maxRot   = qbounds(2);
    maxAdv   = qbounds(3);
            
    % initialize the tree and the starting point
    qListNormalized = zeros(3, nPoints);
    qList = zeros(3, nPoints);
    pList = zeros(3, nPoints);
    aList = zeros(3, nPoints);
    
    % initialize the base transform
    T_robot_in_env = anatomyModel.baseTransform;
    
    % iteratively build the tree
    hw = waitbar(0, 'Sampling the configuration space. Please wait...');
    
    for ii = 1 : nPoints
        qRand = rand(3,1);
        qNearest = nearestVertex(qRand, qListNormalized, ii);
        qNew = move(qNearest, qRand, deltaQ);
        
        % Check for potential collisions
        displ = qNew(1) * maxDispl;  
        rot   = qNew(2) * maxRot;
        adv   = qNew(3) * maxAdv;
%         
        [P,T] = robot.fwkine([displ, rot, adv], T_robot_in_env);
%         P = applytransform(P, T_robot_in_env); %!FIXME T_robot_in_env needs to be calculated
        robotPM = robot.makePhysicalModel(qList(:,500), T_robot_in_env); %!FIXME T_robot_in_env needs to be calculated
        
        testpts = [robotPM.surface.X(:) robotPM.surface.Y(:) robotPM.surface.Z(:)];
        collision = intriangulation(anatomyModel.vertices, ...
            anatomyModel.faces, testpts);
        
        collision = sum(collision);
        
        if collision > 1
            continue;
        end
         
        % If no collision, add this point to the tree
        qListNormalized(:,ii) = qNew;
        
        qList(1,ii) = displ;  
        qList(2,ii) = rot;
        qList(3,ii) = adv;
        
        pList(:,ii) = T(1:3,4,end);
        aList(:,ii) = T(1:3,3,end);
        
        waitbar(ii/nPoints, hw, 'Sampling the configuration space. Please wait...');
    end
    
    close(hw);
end