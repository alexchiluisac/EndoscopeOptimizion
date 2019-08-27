vis = zeros(10,7);
col = distinguishable_colors(10);

for ii = 1 : 10
    for jj = 1:8
       vis(ii,jj) = visibility{ii}(jj);
   end
end

figure, hold on
for ii = 1 : 8
    plot(1:10, vis(:,ii), 'Color', col(ii,:), 'LineWidth', 2.5);
end
xlabel('Number of cutouts');
ylabel('Percentage visible surface');
ylim([0 1]);
legend({'Antrum', 'Epitympanum', 'Eustachian Tube', 'Hypotympanum', 'Sinus Tympanum', 'Supratubal Recess', 'Facial Recess', 'Mesotympanum'});
set(gca,'FontSize',18);
grid on