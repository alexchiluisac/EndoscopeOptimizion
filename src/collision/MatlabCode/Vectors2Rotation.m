function R = Vectors2Rotation(V1, V2)

V1 = V1(:)/norm(V1);
V2 = V2(:)/norm(V2);

th = acos(dot(V1,V2));
Ax = cross(V1, V2);
R = AxisAngle2Rotation(Ax, th);
end