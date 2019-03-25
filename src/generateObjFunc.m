alpha = 0:pi/4:pi;
v = zeros(1,length(alpha));

parfor i = 1:length(alpha)

    v(i) = testModel(alpha);
    
end

scatter(alpha,v,20,'filled');
xlabel('alpha[rad]');
ylabel('Reachable Volume[mm^3]');
grid on
axis tight