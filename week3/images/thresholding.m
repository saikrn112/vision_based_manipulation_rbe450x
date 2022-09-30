close all;
clear all;
clc;

I_color = imread("input_images/WPI_campus.jpeg");
I = rgb2gray(I_color);
[I_sx,I_sy] = size(I);
I_th = zeros(I_sx, I_sy);

th_bw = 200;

% Displays pixels brigter than th_bw as white

for x_i = 1:I_sx
    for y_i = 1:I_sy
        if(I(x_i,y_i)<th_bw)
            I_th(x_i,y_i) = 0;
        else
            I_th(x_i,y_i) = 255;
        end
    end
end

imwrite(I_th,"output_images/image_bw_th.jpeg");