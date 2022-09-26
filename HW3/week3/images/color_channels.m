close all;
clear all;
clc;

I_color = imread('input_images/vegetables.jpeg');

% Generating images for the R, G, B channels

I_red = I_color(:,:,1);
I_green = I_color(:,:,2);
I_blue = I_color(:,:,3);

imwrite(I_red,"output_images/image_red.jpeg");
imwrite(I_green,"output_images/image_green.jpeg");
imwrite(I_blue,"output_images/image_blue.jpeg");