addpath("../anatomical-models/.");
[meModel, T_world2CT, meModelRayCast, meSegmentation] = ...
       loadEarModel('atlas');
figure;
patch(meModel.me);
hold on;
patch(meModel.os);
hold off;