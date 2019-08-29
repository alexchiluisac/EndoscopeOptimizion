% vis = zeros(10,7);
% 
% for ii = 1 : 10
%     for jj = 1:8
%        vis(ii,jj) = visibility{ii}(jj);
%    end
% end

load visibility_results.mat

col = distinguishable_colors(10);

figure, hold on
for ii = 1 : 8
    plot(1:10, vis(:,ii), '-s', 'Color', col(ii,:), 'LineWidth', 2.5);
end
xlabel('Number of cutouts');
ylabel('Percentage visible surface');
ylim([0 1]);
xlim([1 10]);
legend({'Antrum', 'Epitympanum', 'Eustachian Tube', 'Hypotympanum', 'Sinus Tympanum', 'Supratubal Recess', 'Facial Recess', 'Mesotympanum'});
title('atlas');
set(gca,'FontSize',18);
grid on