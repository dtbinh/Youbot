close all;
cell = 0.125;
map_plot_hough = flip(map_plot);
%SE = ones(2,2);
%map_plot_hough = imerode(map_plot_hough, SE); 
[H,T,R] = hough(map_plot_hough);
P  = houghpeaks(H,8,'threshold',ceil(0.3*max(H(:))));

lines = houghlines(map_plot_hough,T,R,P,'FillGap',5,'MinLength',7);
figure, imshow(map_plot_hough), hold on
max_len = 0;
for k = 1:length(lines)
   xy = [lines(k).point1; lines(k).point2];
   plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','green');

   % Plot beginnings and ends of lines
   plot(xy(1,1),xy(1,2),'x','LineWidth',2,'Color','yellow');
   plot(xy(2,1),xy(2,2),'x','LineWidth',2,'Color','red');

   % Determine the endpoints of the longest line segment
   len = norm(lines(k).point1 - lines(k).point2);
   if ( len > max_len)
      max_len = len;
      xy_long = xy;
   end
   map_plot_hough(xy(1,2):xy(2,2),xy(1,1):xy(2,1)) = false;
end
title('Hough result');

figure;
imshow(map_plot_hough)
title('After removing lines of Hough ');
SE = ones(2,2); % If no erode on line 4 and 5
map_plot_hough = imerode(map_plot_hough, SE);
figure;
imshow(map_plot_hough)
title('Erosion of previous Figure');
SE = strel('disk',2);
%SE = ones(2,2);
map_plot_hough = imdilate(map_plot_hough, SE);
figure;
imshow(map_plot_hough)
title('Dilation of the previous Figure');
map_plot_hough_bis =imresize(map_plot_hough,4);
[centers, radii, ~] = imfindcircles(map_plot_hough_bis,[13 30], 'ObjectPolarity','bright', ...
    'Sensitivity',0.976, 'EdgeThreshold',0.93);
%d = imdistline;
% We only take the 7 first strongest circles
%centers = centers(1:7,:);
%radii = radii(1:7);

centers = round(centers(:,:)/4);
radii = round(radii/4);
figure,
imshow(flip(map_plot)); 
hold on;
plot(centers(:,1), centers(:,2),'.g');
viscircles(centers, radii,'EdgeColor','b');
title('Final result');