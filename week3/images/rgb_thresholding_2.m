close all;
clear all;
clc;

I = imread('input_images/vegetables.jpeg');
[I_sx,I_sy,channel_no] = size(I);
I_th = zeros(I_sx, I_sy);

th_red = 200;
th_blue = 50;
th_green = 50;

% thresholding pixels that has more high values (more than th_red) in 
% red channel and low values in the other channels
for x_i = 1:I_sx
    for y_i = 1:I_sy
        if(I(x_i,y_i,1)>th_red && I(x_i,y_i,2)<th_blue && I(x_i,y_i,3)<th_green)
            I_th(x_i,y_i) = 0;
        else
            I_th(x_i,y_i) = 255;
        end
    end
end

imwrite(I_th,"output_images/image_red_only_th.jpeg");