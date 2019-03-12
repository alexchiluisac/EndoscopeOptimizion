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
    
    nPoints = 200;
    
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
 
    [~,T] = robot.fwkine(qList(:,1), T_robot_in_env);
    pList(:,1) = T(1:3,4,end);
    aList(:,1) = T(1:3,3,end);
    % iteratively build the tree
    hw = waitbar(0, 'Sampling the configuration space. Please wait...');
    
    jj = 2;
    
    for ii = 1 : nPoints
        qRand = rand(3,1);
        qNearest = nearestVertex(qRand, qListNormalized, jj);
        qNew = move(qNearest, qRand, deltaQ);
        
        % Check for potential collisions
        displ = qNew(1) * maxDispl;  
        rot   = qNew(2) * maxRot;
        adv   = qNew(3) * maxAdv;
%         
        [P,T] = robot.fwkine([displ, rot, adv], T_robot_in_env);
%         P = applytransform(P, T_robot_in_env); %!FIXME T_robot_in_env needs to be calculated
        robotPM = robot.makePhysicalModel([displ, rot, adv], T_robot_in_env); %!FIXME T_robot_in_env needs to be calculated
        
        testpts = [robotPM.surface.X(:) robotPM.surface.Y(:) robotPM.surface.Z(:)];
        collision = intriangulation(anatomyModel.vertices, ...
            anatomyModel.faces, testpts);
        
        collision = sum(collision);
        
        if collision > 1
            disp('Collision');
            continue;
        end
         
        % If no collision, add this point to the tree
        qListNormalized(:,jj) = qNew;
        
        qList(1,jj) = displ;  
        qList(2,jj) = rot;
        qList(3,jj) = adv;
        
        pList(:,jj) = T(1:3,4,end);
        aList(:,jj) = T(1:3,3,end);
        
        jj = jj + 1;
        
        waitbar(ii/nPoints, hw, 'Sampling the configuration space. Please wait...');
    end
    
    close(hw);
    
    qListNormalized = qListNormalized(:,1:jj-1);
    qList = qList(:,1:jj-1);
    pList = pList(:,1:jj-1);
    aList = aList(:,1:jj-1);
end