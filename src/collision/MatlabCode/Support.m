function varargout = Support(S1, S2, d)

p1 = getFarthestPointInDirection(S1, d);
p2 = getFarthestPointInDirection(S2, -d);
p3 = p1 - p2;

if nargout > 0
    varargout{1} = p3;
end
if nargout > 1
    varargout{2} = p1;
end
if nargout > 2
    varargout{3} = p2;
end
end

function FarthestPointInDirection = getFarthestPointInDirection(S, d)
[~, idx] = max(d.'*S);
FarthestPointInDirection = S(:,idx);
end