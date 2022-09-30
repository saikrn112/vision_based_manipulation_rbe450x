close all;
clear all;
clc;

I = imread('input_images/vegetables.jpeg');
[I_sx,I_sy,channel_no] = size(I);
I_th = zeros(I_sx, I_sy);

th_red = 200;

% thresholding pixels that has more high values (more than th_red) in 
% red channel
for x_i = 1:I_sx
    for y_i = 1:I_sy
        if(I(x_i,y_i,1)>th_red)
            I_th(x_i,y_i) = 0;
        else
            I_th(x_i,y_i) = 255;
        end
    end
end

imwrite(I_th,"output_images/image_red_th.jpeg");