function R = calculateRotation(a, b)
% CALCULATEROTATION returns the rotation matrix R required to align unit
% vector a with unit vector b

v = cross(a,b);
c = dot(a,b);

Vx = [ 0    -v(3)   v(2);
       v(3)  0     -v(1);
      -v(2)  v(1)   0];

R = eye(3) + Vx + Vx^2 / (1 + c);
end