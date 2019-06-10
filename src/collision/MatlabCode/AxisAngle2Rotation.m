function R = AxisAngle2Rotation(A, th)

if norm(A) ~= 1
    A = A/norm(A);
end

OMCTh = 1-cos(th);
Sth = sin(th);
Cth = cos(th);

R = [Cth+A(1)^2*OMCTh,          A(1)*A(2)*OMCTh-A(3)*Sth, A(1)*A(3)*OMCTh+A(2)*Sth;
     A(2)*A(1)*OMCTh+A(3)*Sth,  Cth+A(2)^2*OMCTh,         A(2)*A(3)*OMCTh-A(1)*Sth;
     A(3)*A(1)*OMCTh-A(2)*Sth,  A(3)*A(2)*OMCTh+A(1)*Sth, Cth+A(3)^2*OMCTh];
end