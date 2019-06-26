function [t,flag] = rayTriangleIntersection_gpu (oox,ooy,ooz,...
    ox,oy,oz,...
    dx,dy,dz,...
    p0x,p0y,p0z,...
    p1x,p1y,p1z,...
    p2x,p2y,p2z)

% DESCRIPTION: This function performs raycasting on the GPU

% INPUTS
% [oox,ooy,ooz]: Vector of sheath origin coordinates 
% [ox,oy,oz]: Vector of sheath end-effector coordinates 
% [dx,dy,dz]: Sheath end-effector direction vector
% [p0x,p0y,p0z]: Vector of vertex 1 coordinates
% [p1x,p1y,p1z]: Vector of vertex 2 coordinates
% [p2x,p2y,p2z]: Vector of vertex 3 coordinates
%
% OUTPUTS
% t: Intersection point [3x1]
% flag: binary for collision detection (1 if collision, 0 if not)
%
% Written By: Joshua Gafford
% Last Edits: 03/01/2019


length = 30;

% Ray/triangle intersection using the algorithm proposed by Miller
% and Trumbore (1997).
epsilon = 0.00001;

% Edge 1 computation
e1x = p1x-p0x;
e1y = p1y-p0y;
e1z = p1z-p0z;

% Edge 2 computation
e2x = p2x-p0x;
e2y = p2y-p0y;
e2z = p2z-p0z;


% Triangle normal
[cx,cy,cz] = cross_product(e1x,e1y,e1z,e2x,e2y,e2z);


% q  = cross(d,e2);
% p_vec from other code
[qx,qy,qz] = cross_product(dx,dy,dz,e2x,e2y,e2z);

% a  = dot(e1,q); % determinant of the matrix M
a = e1x*qx + e1y*qy + e1z*qz;
t = inf;    % Initialize intersection point at infinity

% if vertex is beyond reach of the sheath
if (sqrt((ox-p0x)^2+(oy-p0y)^2+(oz-p0z)^2)>length)
    flag = false;
    return
end

% Treat triangle as single-sided (flag is false if the direction vector
% is opposing the triangle normal)
if (dx*cx+dy*cy+dz*cz < 0)
    flag = false;
    return
end

% if vertex is behind the start of the sheath (assuming sheath is
% aligned on the x-axis)
if (p1x<oox)
    flag = false;
    return
end

if (a>-epsilon && a<epsilon)
    % the vector is parallel to the plane (the intersection is at infinity)
    flag = false;
    return
end

f = 1/a;
% s = o-p0;

% t_Vec from other code
sx = ox - p0x;
sy = oy - p0y;
sz = oz - p0z;


% u = f*dot(s,q);
% 1st barycentric coordinate
u = f*(sx*qx + sy*qy + sz*qz);

if (u<0.0)
    % the intersection is outside of the triangle
    flag = false;
    return
end

% r = cross(s,e1);
% q_vec from other code
[rx,ry,rz] = cross_product(sx,sy,sz,e1x,e1y,e1z);

% v = f*dot(d,r);
% 2nd barycentric coordinate
v = f*(dx*rx + dy*ry + dz*rz);

if (v<0.0 || u+v>1.0)
    % the intersection is outside of the triangle
    flag = false;
    return;
end


angleOK = (abs(a)>epsilon);        % treat triangle as one-sided
ok   = (angleOK & u>=0 & v>0 & u+v<=1.0);

% Compute intersection point.
t = f*(e2x*rx+e2y*ry+e2z*rz);

if t<=0||~ok
    % remove intersections behind ray
    flag=false;
    return;
end

flag = true;
return;
end
function [w1,w2,w3] = cross_product(u1,u2,u3,v1,v2,v3)
w1 = u2*v3 - u3*v2;
w2 = u3*v1 - u1*v3;
w3 = u1*v2 - u2*v1;
end