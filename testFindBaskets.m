close all;
cell = 0.125;
%map_bis = flip(+(~map));
SE = ones(7,7);
%SE = strel('rectangle',[1,1]);
map_bis = imdilate(flip(+map), SE); % + transfo in double
map_bis = imresize(map_bis,4);
map_bis(map_bis == 1) = 255;
 
%subplot(121), imshow(flip(map));
%subplot(122), imshow(map_bis);
figure;
imshow(flip(map));
figure;
imshow(map_bis);
 
% radius_min = floor(0.750/cell);
% radius_max = floor(0.850/cell);
% 
[centers, radii, ~] = imfindcircles(map_bis,[13 30], 'ObjectPolarity','bright', ...
    'Sensitivity',0.97, 'EdgeThreshold',0.93);
hold on;
plot(floor(centers(:,1)), floor(centers(:,2)),'.g');
viscircles(centers, radii,'EdgeColor','b');

figure,
imshow(flip(map)); 
hold on;
plot(floor(centers(:,1)/4), floor(centers(:,2)/4),'.g');
viscircles(floor(centers/4), floor(radii/4),'EdgeColor','b');

