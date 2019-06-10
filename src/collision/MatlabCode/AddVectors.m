function vr = AddVectors(v1,v2)

if size(v1, 2) == 1
    vr = repmat(v1, 1, size(v2, 2)) + v2;
elseif size(v2, 2) == 1
    vr = v1 + repmat(v2, 1, size(v1, 2));
else
    vr = v1 + v2;
end
