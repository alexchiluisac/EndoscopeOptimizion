function varargout = ExtractSubscripts(A, varargin)

if any(strcmpi(varargin, 'Equal'))
    Equal = varargin{find(strcmpi(varargin, 'Equal'))+1};
else
    Equal = 1;
end

if any(strcmpi(varargin, 'GreaterThan'))
    GreaterThan = varargin{find(strcmpi(varargin, 'GreaterThan'))+1};
else
    GreaterThan = [];
end
if any(strcmpi(varargin, 'LessThan'))
    LessThan = varargin{find(strcmpi(varargin, 'LessThan'))+1};
else
    LessThan = [];
end
if any(strcmpi(varargin, 'Columns'))
    Columns = varargin{find(strcmpi(varargin, 'Columns'))+1};
else
    Columns = false;
end

if ~isempty(GreaterThan)
    Inds = find(A>GreaterThan);
elseif ~isempty(LessThan)
    Inds = find(A<LessThan);
else
    Inds = find(A==Equal);
end


if Columns && ~isempty(Inds)
    [Subs(:,2), Subs(:,1), Subs(:,3)] = ind2sub(size(A), Inds);
elseif ~isempty(Inds)
    [Subs(2,:), Subs(1,:), Subs(3,:)] = ind2sub(size(A), Inds);
else
    Subs = [];
end

varargout{1} = Subs;
if nargout>1
    varargout{2} = Inds;
end