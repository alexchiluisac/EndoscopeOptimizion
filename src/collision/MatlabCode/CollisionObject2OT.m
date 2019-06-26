function [Collision, FinalCollision] = CollisionObject2OT(CurrentParent, OT, BinCorners, BinCenter, MethodGJK, Plotting)
if nargin < 5, MethodGJK = true;end
if nargin < 6, Plotting = false;end

FinalCollision = false;
OTSiblingInds = find(OT.BinParents == CurrentParent);

for ii = 1:length(OTSiblingInds)

    if MethodGJK
        Collision = CollisonGJK(OT.BinCorners(:,:,OTSiblingInds(ii)), BinCorners, Plotting);
    else
        Collision = CollisionOABB2OT(OT.BinCorners(:,:,OTSiblingInds(ii)), OT.Centers(OTSiblingInds(ii),:), BinCorners, BinCenter, MethodGJK, Plotting);
    end
    
    if Collision && OT.BinDepths(OTSiblingInds(1)) == OT.MaxBinDepth
        FinalCollision = true;
        return
    elseif Collision
        [Collision, FinalCollision] = CollisionObject2OT(OTSiblingInds(ii), OT, BinCorners, BinCenter, MethodGJK, Plotting);
        if FinalCollision
            return
        end
    end
end