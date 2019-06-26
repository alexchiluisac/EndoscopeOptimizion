function varargout = PlotPoints(Points, ColumnPoints, varargin)
% Michael Siebold
% 3/15/2017
%
% 'Points' is a 3xmxk or nx3xk sized array of points
%          or a 2xmxk or nx2xk sized array of points
%
% ColumnPoints -> logical flag only takes over when Points is 3x3xk
%                 True (default)  -> points are stored as 3xmxk
%                 False           -> Points are stored as nx3xk
%
% varargin is passed to plot3 and should be used for plotting parameters 
%
% 'Points' may be up to 3 dimensional
if nargin < 2||isempty(ColumnPoints), ColumnPoints = true;end
if nargin < 3
    varargin = {'linestyle', 'none', 'marker', '.'};
elseif ~any(strcmpi(varargin, 'linestyle'))
    varargin = {varargin{:}, 'linestyle', 'none'}; %#ok<CCAT>
end

[n,m,k] = size(Points);

if m == 3 || m == 2 && ~ColumnPoints
    Points = permute(Points, [2,1,3]);
end

if k ~= 1
    Points = reshape(Points, [3, numel(Points)/3, 1]);
end

if size(Points,1) == 2
    handle = plot(Points(1,:), Points(2,:), varargin{:});
elseif size(Points,1) == 3
    handle = plot3(Points(1,:), Points(2,:), Points(3,:), varargin{:});
end

if nargout > 0
    varargout{1} = handle;
end