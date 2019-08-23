vol = zeros(1,7);

for ii = 3 : 10
    load([num2str(ii) '-simulation.mat'])
    vol(ii-2) = v .* 1e9;
    v
end

plot(3:10, vol, 'b', 'LineWidth', 2.5);
xlabel('Number of cutouts');
ylabel('Reachable Volume [mm^3]');
set(gca,'FontSize',18);
grid on