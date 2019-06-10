function MinkDiff = MinkowskiDifference(A, B)

AA = repmat(A, 1, size(B, 2));
BB = reshape(repmat(B, size(A, 2), 1), 3, size(A, 2)*size(B, 2));
MinkDiff = AA - BB;