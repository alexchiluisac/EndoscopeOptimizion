vol = zeros(1,3);

for ii = 1 : 10
    load([num2str(ii) '-simulation.mat'])
    vol(ii) = v .* 1e9;
    v
end

figure
plot(1:10, vol, 'b', 'LineWidth', 2.5);
xlabel('Number of cutouts');
ylabel('Reachable Volume [mm^3]');
set(gca,'FontSize',18);
grid on