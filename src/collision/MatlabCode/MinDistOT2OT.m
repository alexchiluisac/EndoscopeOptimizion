function [MD, cpS1, cpS2] = MinDistOT2OT(CurrentParent, OT1, OT2, Plotting)

if nargin < 4, Plotting = false;end

OT1SiblingInds = find(OT1.BinParents == CurrentParent);

for ii = 1:length(OT1SiblingInds)
    [MDList(ii), cpS1List(:,ii), cpS2List(:,ii)] = MinDistObj2OT(CurrentParent, OT2, OT1.BinCorners(:,:,OT1SiblingInds(ii)), Plotting);
end

[MD, idx] = min(MDList);
cpS1 = cpS1List(:,idx);
cpS2 = cpS2List(:,idx);


if OT1.BinDepths(OT1SiblingInds(1)) ~= OT1.MaxBinDepth
    [MD, cpS1, cpS2] = MinDistOT2OT(OT1SiblingInds(idx), OT1, OT2, Plotting);
end