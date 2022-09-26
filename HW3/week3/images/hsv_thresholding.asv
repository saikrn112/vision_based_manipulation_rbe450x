clear all;
close all;
clc;

I_rgb = imread('input_images/vegetables.jpeg');
I = rgb2hsv(I_rgb);
[I_sx,I_sy,channel_no] = size(I);
I_th = zeros(I_sx, I_sy);

th_h_high = 0.1;
th_h_low = 0.05;
th_s = 0.1;
th_v = 0.1;

% thresholding pixels in HSV space
for x_i = 1:I_sx
    for y_i = 1:I_sy
        if((I(x_i,y_i,1)<th_h_high && I(x_i,y_i,1)>th_h_low) && I(x_i,y_i,2)>th_s && I(x_i,y_i,3)>th_v)
            I_th(x_i,y_i) = 0;
        else
            I_th(x_i,y_i) = 255;
        end
    end
end

imwrite(I_th,"output_images/image_hsv_red_th.jpeg");