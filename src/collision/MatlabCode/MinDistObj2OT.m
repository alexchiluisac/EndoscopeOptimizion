function [MD, cpS1, cpS2] = MinDistObj2OT(CurrentParent, OT, ObjPoints, Plotting)

if nargin < 4, Plotting = false;end

OTSiblingInds = find(OT.BinParents == CurrentParent);

for ii = 1:length(OTSiblingInds)
    [MD(ii), cpS1(:,ii), cpS2(:,ii)] = MinDistGJK(OT.BinCorners(:,:,OTSiblingInds(ii)), ObjPoints, [], [], Plotting);
end

[MD, idx] = min(MD);
cpS1 = cpS1(:,idx);
cpS2 = cpS2(:,idx);

if OT.BinDepths(OTSiblingInds(1)) ~= OT.MaxBinDepth
    [MD, cpS1, cpS2] = MinDistObj2OT(OTSiblingInds(idx), OT, ObjPoints, Plotting);
end