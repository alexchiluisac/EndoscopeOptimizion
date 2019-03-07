function [qListNormalized,qList,pList,aList] = rrt(robot, qbounds, q0, anatomyModel)
% RRT implements the basic Rapidly-Exploring Random Trees algorithm for a
% generic continuum robot
%
% Author: L. Fichera <lfichera@wpi.edu>
%
% Last revision: 3/6/2019

    if nargin < 4
        collisionDetection = false;
    else
        collisionDetection = true;
    end
    
    nPoints = 5000;
    
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
    
    % iteratively build the tree
    hw = waitbar(0, 'Sampling the configuration space. Please wait...');
    
    for ii = 1 : nPoints
        qRand = rand(3,1);
        qNearest = nearestVertex(qRand, qListNormalized, ii);
        qNew = move(qNearest, qRand, deltaQ);
%         
%         qNew(1) = qNew(1);
%         qNew(2) = qNew(2);
%         qNew(3) = qNew(3);
        
        qListNormalized(:,ii) = qNew;
        
        qList(1,ii) = qNew(1) * maxDispl;  
        qList(2,ii) = qNew(2) * maxRot;
        qList(3,ii) = qNew(3) * maxAdv;
        
        [~,T] = robot.fwkine(qList(:,ii));
        pList(:,ii) = T(1:3,4,end);
        aList(:,ii) = T(1:3,3,end);
        
        waitbar(ii/nPoints, hw, 'Sampling the configuration space. Please wait...');
    end
    
    close(hw);
end
% 
% %nLinks, maxL, nPoints, anatomyModel, baseTransform)
% % RRT implements the basic Rapidly-Exploring Random Trees algorithm in a
% % 3*n-dimensional configuration space.
% %
% % Author: L. Fichera <lfichera@wpi.edu>
% %
% % Last revision: 10/12/2018
% 
%     if nargin < 5
%         collisionDetection = false;
%     else
%         collisionDetection = true;
%     end
%         
%     % algorithm parameters
%     deltaQ = repmat([0.05 0.05 0.05]', nLinks, 1);    
%     maxPhi = 4 * pi; % [rad]
%     maxK   = 0.1;    % [mm^-1]
%     minL   = 0;      % [mm]
%     
%     % initialize the tree and the starting point   
%     qList = zeros(3 * nLinks, nPoints);
%     
%     % initialize the tree and the starting point   
%     graph = zeros(3 * nLinks, nPoints);
%     
%     pList = zeros(3, nPoints);
%     aList = zeros(3, nPoints);
%     aList(:,1) = [0 0 1]';
%     
%     % iteratively build the tree
%     hw = waitbar(0,'Sampling the configuration space. Please wait...');
%     
%     ii = 1;
%     jj = 0;
%     
%     while (true)
%     %for i = 2 : maxIterations
%         % generate a random configuration
%         qRand = rand(3 * nLinks,1);
%         qNearest = nearestVertex(qRand, graph, ii);
%         qNew = move(qNearest, qRand, deltaQ, nLinks);
%         graphPoint = qNew;
%         
%         for j = 0 : nLinks - 1
%             qNew(1+(j*3)) = qNew(1+(j*3)) * maxK;
%             qNew(2+(j*3)) = qNew(2+(j*3)) * maxPhi;
%             qNew(3+(j*3)) = minL +  (maxL - minL) * qNew(3+(j*3));            
%         end        
%         
%         % Check the points fall within permissible range
%         
%         if qNew(1) > maxK || qNew(2) > maxPhi || qNew(3) > maxL
%             jj = jj + 1;
%             continue
%         end
%         
%         if qNew(1) < 0 || qNew(2) < 0 || qNew(3) < minL
%             jj = jj + 1;
%             continue
%         end
%         
%         if nLinks == 2
%            if qNew(4) > maxK || qNew(5) > maxPhi || qNew(6) > maxL
%                jj = jj + 1;
%                 continue
%            end
%         
%             if qNew(4) < 0 || qNew(5) < 0 || qNew(6) < minL
%                 jj = jj + 1;
%                 continue
%             end 
%         end
%         
%         
%         waitbar(ii/nPoints, hw, 'Sampling the configuration space. Please wait...');
%                 
%         if collisionDetection
%             [T1, link] = arckinematics(qNew(1:3,1), true);
%             link = applytransform(link, baseTransform);
%             
%             [X, Y, Z] = gencyl(link(1:3,:), 1.8 / 2 * ones(1,size(link,2)));
%             
%             c1 = any(intriangulation(anatomyModel.vertices, ...
%                 anatomyModel.faces, ...
%                 [X(:) Y(:) Z(:)]));
%             
%             if nLinks == 2
%                 [~, link2] = arckinematics(qNew(4:6,1), true);
%                 link2 = applytransform(link2, T1);
%                 link2 = applytransform(link2, baseTransform);
%                 
%                 [X, Y, Z] = gencyl(link2(1:3,:), 1.4 / 2 * ones(1,size(link,2)));
%                 
%                 c2 = any(intriangulation(anatomyModel.vertices, ...
%                     anatomyModel.faces, ...
%                     [X(:) Y(:) Z(:)]));
%             end
%             
%             
%             if nLinks == 1
%                 if c1
%                     continue;
%                 end
%             elseif nLinks == 2
%                 if c1 || c2
%                     continue;
%                 end
%             end
%         end
%         
%         ii = ii + 1;
%         T = robot.fwkinematics(qNew);
%         pList(:,ii) = T(1:3,4);
%         aList(:,ii) = T(1:3,3);
%         qList(:,ii) = qNew;
%         graph(:,ii) = graphPoint;
%         
%         if ii == nPoints
%             break;
%         end         
%     end
%         
%      close(hw);
%      jj
% end