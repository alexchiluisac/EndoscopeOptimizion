function [Collision, FinalCollision] = CollisionOT2OT(CurrentParent, OT1, OT2, Plotting)
if nargin < 4, Plotting = false;end

FinalCollision = false;
OT1SiblingInds = find(OT1.BinParents == CurrentParent);

for ii = 1:length(OT1SiblingInds)
    
    Collision = CollisionOABB2OT(0, OT2, OT1.BinCorners(:,:,OT1SiblingInds(ii)), OT1.BinCenters(OT1SiblingInds(ii),:), false, Plotting);
    
    if Collision && OT1.BinDepths(OT1SiblingInds(1)) == OT1.MaxBinDepth
        FinalCollision = true;
        return
    elseif Collision
        [Collision, FinalCollision] = CollisionOT2OT(OT1SiblingInds(ii), OT1, OT2, Plotting);
        if FinalCollision
            return
        end
    end
end
end