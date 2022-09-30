clear all;
close all;
clc;

th = 0.25;

I_rgb = imread('input_images/hand_object.png');
I = rgb2gray(I_rgb);
I_edge = edge(I,'sobel',th);
[H,theta,rho] = hough(I_edge);
P = houghpeaks(H,20,'threshold',ceil(0.25*max(H(:))));

lines = houghlines(I_edge,theta,rho,P,'FillGap',5,'MinLength',50);

figure, imshow(I_rgb), hold on
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
end

%imwrite(I_edge,"output_images/hand_lines.jpeg");