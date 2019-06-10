function H = hat(W)
if length(W)==6
    H = [hat(W(4:6)),W(1:3);zeros(1,4)];
elseif length(W)==3
    H = [0,-W(3),W(2);W(3),0,-W(1);-W(2),W(1),0];
end
end