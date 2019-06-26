function tp = TripleProduct(a, b, c)
% (axb)xc
tp = c.'*a*b - c.'* b*a;
end